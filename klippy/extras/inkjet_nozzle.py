# Klipper extra module: HPC6602 inkjet printhead support for BinderJetting
#
# Replicates the Marlin HPC6602 mode:
#   – G1 X<val> Y<val> Z<val> E<val> S<0-4095>
#     S is a 12-bit nozzle bitmask; each bit selects one of the 12 nozzles.
#     S=4096 (or omitted) means "no fire" (useful for travel moves or E1 use).
#   – E0 axis is the inkjet axis: each "extruder step" fires the selected
#     nozzles via 4 address pins + 1 pulse pin wired to the HPC6602 head.
#   – E1 axis is the powder-feed stepper (configured as [manual_stepper powder_feed]).
#
# Physical buttons (3-position switch + UP/DOWN buttons):
#   BTN_UP   / BTN_DOWN  : directional buttons (active-low, on RAMPS AUX-2)
#   BTN_LEFT             : 3-pos switch LEFT  → jog E1 (powder feed)
#   BTN_RIGHT            : 3-pos switch RIGHT → jog Z  (build piston)
#   Centre (both open)   : no action
#
# Configuration (printer.cfg):
#   [inkjet_nozzle]
#   pin_a: ar53       # HPC6602 A0 address bit
#   pin_b: ar52       # HPC6602 A1
#   pin_c: ar51       # HPC6602 A2
#   pin_d: ar50       # HPC6602 A3
#   pin_pulse: ar49   # HPC6602 fire / strobe
#   inkjet_stepper: extruder   # name of the [extruder] section for E0
#   manual_move_step: 0.1      # mm per button-press event
#   manual_move_speed: 5.0     # mm/s
#
# Copyright (C) 2024  BinderJet project
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging

# Sentinel value: "no fire" (matches Marlin's 4096 strip_code sentinel)
NOZZLE_CODE_IDLE = 4096


class InkjetNozzle:
    """
    Manages the HPC6602 inkjet printhead on a Klipper-driven BinderJet printer.

    Startup sequence
    ----------------
    1. __init__:  read config, register MCU config callback via pin lookup.
    2. _build_config (MCU config phase):
       • resolve inkjet pin numbers,
       • look up the extruder stepper OID (already assigned by MCU_stepper),
       • send ``config_inkjet_nozzle`` MCU command,
       • look up ``stepper_set_nozzle_code`` MCU command for runtime use.
    3. klippy:connect:  register G0/G1 overrides (toolhead is ready by now).

    Runtime
    -------
    • G1 with S<code> → store nozzle_code, call original G1, register lookahead
      callback that sends clock-synchronised ``stepper_set_nozzle_code`` to the
      MCU exactly 1 ms before the move's first E0 step fires.
    • Physical buttons → immediate jogging of Z or E1 via run_script_from_command.
    """

    def __init__(self, config):
        self.printer = config.get_printer()
        self.name    = config.get_name()

        # --- Inkjet pin names (resolved to MCU pin numbers in _build_config) --
        self._pin_a_name     = config.get('pin_a')
        self._pin_b_name     = config.get('pin_b')
        self._pin_c_name     = config.get('pin_c')
        self._pin_d_name     = config.get('pin_d')
        self._pin_pulse_name = config.get('pin_pulse')

        # Name of the [extruder] / extruder section that is the E0 inkjet axis
        self._inkjet_stepper_name = config.get('inkjet_stepper', 'extruder')

        # Manual-move parameters
        self.manual_step  = config.getfloat('manual_move_step',  0.1, above=0.)
        self.manual_speed = config.getfloat('manual_move_speed', 5.0, above=0.)

        # Runtime nozzle pattern (0..4095 = specific nozzles, 4096 = idle)
        self.nozzle_code = NOZZLE_CODE_IDLE

        # These are filled in during _build_config
        self._mcu                 = None
        self._inkjet_oid          = None
        self._set_nozzle_code_cmd = None

        # Button state (True = currently pressed)
        self._btn_up    = False
        self._btn_down  = False
        self._btn_left  = False   # 3-pos: E1 (powder feed) mode
        self._btn_right = False   # 3-pos: Z (build piston) mode

        # --- Determine the MCU from the pin_a configuration -----------------
        # This also validates the pin name early and lets us register our
        # config callback on the correct MCU.
        ppins = self.printer.lookup_object('pins')
        pin_params = ppins.lookup_pin(self._pin_a_name,
                                      can_invert=False, can_pullup=False)
        self._mcu = pin_params['chip']

        # Register MCU config callback – fires during the MCU config phase
        # (before klippy:connect), after all stepper OIDs are assigned.
        self._mcu.register_config_callback(self._build_config)

        # --- klippy:connect: register G-code command overrides ---------------
        # We do this here rather than in _build_config because the gcode / move
        # objects are not guaranteed to be available during the MCU config phase.
        self.printer.register_event_handler(
            'klippy:connect', self._handle_connect)

    # ------------------------------------------------------------------
    # MCU config phase – send inkjet setup command
    # ------------------------------------------------------------------

    def _build_config(self):
        """Called by the MCU during its config phase (before klippy:connect)."""
        ppins = self.printer.lookup_object('pins')

        def resolve_pin(name):
            params = ppins.lookup_pin(name, can_invert=False, can_pullup=False)
            if params['chip'] is not self._mcu:
                raise self.printer.config_error(
                    "inkjet_nozzle: pin '%s' is on a different MCU than pin_a" % name)
            return params['pin']

        pin_a     = resolve_pin(self._pin_a_name)
        pin_b     = resolve_pin(self._pin_b_name)
        pin_c     = resolve_pin(self._pin_c_name)
        pin_d     = resolve_pin(self._pin_d_name)
        pin_pulse = resolve_pin(self._pin_pulse_name)

        # Look up the E0 extruder stepper.  All config sections are loaded
        # before build_config callbacks run, so lookup_object is safe here.
        extruder = self.printer.lookup_object(self._inkjet_stepper_name, None)
        if extruder is None:
            raise self.printer.config_error(
                "inkjet_nozzle: cannot find stepper '%s'"
                % self._inkjet_stepper_name)
        try:
            mcu_stepper = extruder.extruder_stepper.stepper
        except AttributeError:
            raise self.printer.config_error(
                "inkjet_nozzle: '%s' does not have an extruder stepper"
                % self._inkjet_stepper_name)

        if mcu_stepper.get_mcu() is not self._mcu:
            raise self.printer.config_error(
                "inkjet_nozzle: E0 stepper must be on the same MCU as the"
                " inkjet pins")

        self._inkjet_oid = mcu_stepper.get_oid()

        # Send the one-time MCU configuration command.
        # This converts the E0 stepper's step-event handler to inkjet-fire mode.
        self._mcu.add_config_cmd(
            "config_inkjet_nozzle oid=%d pin_a=%s pin_b=%s pin_c=%s"
            " pin_d=%s pin_pulse=%s" % (
                self._inkjet_oid,
                pin_a, pin_b, pin_c, pin_d, pin_pulse))

        # Also re-disable inkjet on every firmware restart (e.g. after M112)
        self._mcu.add_config_cmd(
            "stepper_set_nozzle_code oid=%d clock=0 nozzle_code=%d"
            % (self._inkjet_oid, NOZZLE_CODE_IDLE),
            on_restart=True)

        # Look up the runtime nozzle-code update command
        cq = self._mcu.alloc_command_queue()
        self._set_nozzle_code_cmd = self._mcu.lookup_command(
            "stepper_set_nozzle_code oid=%c clock=%u nozzle_code=%hu",
            cq=cq)

        logging.info(
            "inkjet_nozzle: E0 stepper oid=%d configured in inkjet mode "
            "(pins a=%s b=%s c=%s d=%s pulse=%s)",
            self._inkjet_oid, pin_a, pin_b, pin_c, pin_d, pin_pulse)

    # ------------------------------------------------------------------
    # klippy:connect – wire up G-code command overrides and buttons
    # ------------------------------------------------------------------

    def _handle_connect(self):
        gcode = self.printer.lookup_object('gcode')
        self.gcode_move = self.printer.lookup_object('gcode_move')

        # Override G0 and G1 to intercept the S (nozzle-code) parameter.
        # We replace the commands and delegate back to gcode_move.cmd_G1.
        gcode.register_command('G0', self._cmd_G1)
        gcode.register_command('G1', self._cmd_G1)

        # Manual control commands
        gcode.register_command(
            'SET_NOZZLE_CODE', self._cmd_SET_NOZZLE_CODE,
            desc=self._cmd_SET_NOZZLE_CODE_help)

        # Button commands (invoked by [gcode_button] press/release_gcode)
        for name, fn in (
            ('_INKJET_BTN_UP_PRESS',      self._btn_up_press),
            ('_INKJET_BTN_UP_RELEASE',    self._btn_up_release),
            ('_INKJET_BTN_DOWN_PRESS',    self._btn_down_press),
            ('_INKJET_BTN_DOWN_RELEASE',  self._btn_down_release),
            ('_INKJET_BTN_LEFT_PRESS',    self._btn_left_press),
            ('_INKJET_BTN_LEFT_RELEASE',  self._btn_left_release),
            ('_INKJET_BTN_RIGHT_PRESS',   self._btn_right_press),
            ('_INKJET_BTN_RIGHT_RELEASE', self._btn_right_release),
        ):
            gcode.register_command(name, fn)

    # ------------------------------------------------------------------
    # G0 / G1 override – intercept S parameter
    # ------------------------------------------------------------------

    def _cmd_G1(self, gcmd):
        # Extract optional S parameter (nozzle bitmask 0..4095)
        s_val = gcmd.get_int('S', None, minval=0, maxval=4095)
        self.nozzle_code = s_val if s_val is not None else NOZZLE_CODE_IDLE

        # Snapshot for the lambda (avoids capturing the mutable attribute)
        code = self.nozzle_code

        # Let gcode_move handle X/Y/Z/E/F as usual
        self.gcode_move.cmd_G1(gcmd)

        # Attach a lookahead callback to the move we just queued.
        # It fires when the toolhead is dispatching that move, giving us the
        # exact print_time so we can clock-synchronise the MCU nozzle update.
        if self._set_nozzle_code_cmd is not None:
            toolhead = self.printer.lookup_object('toolhead')
            toolhead.register_lookahead_callback(
                lambda pt: self._apply_nozzle_code(pt, code))

    def _apply_nozzle_code(self, print_time, code):
        """Send a clock-synchronised nozzle_code update to the MCU.

        Schedules the update 1 ms before the move starts, so the pattern is
        always set before the first E0 step event fires.
        """
        target_time = max(print_time - 0.001, 0.)
        clock = self._mcu.print_time_to_clock(target_time)
        self._set_nozzle_code_cmd.send([self._inkjet_oid, clock, code])

    # ------------------------------------------------------------------
    # SET_NOZZLE_CODE – immediate manual override
    # ------------------------------------------------------------------

    _cmd_SET_NOZZLE_CODE_help = (
        "Set the HPC6602 nozzle firing pattern directly. "
        "CODE=0..4095 selects nozzles; CODE=4096 disables firing.")

    def _cmd_SET_NOZZLE_CODE(self, gcmd):
        code = gcmd.get_int('CODE', None, minval=0, maxval=4096)
        if code is None:
            gcmd.respond_info("Current nozzle code: 0x%03X (%d)"
                              % (self.nozzle_code & 0xFFF, self.nozzle_code))
            return
        self.nozzle_code = code
        if self._set_nozzle_code_cmd is not None:
            # clock=0 → apply immediately on the MCU
            self._set_nozzle_code_cmd.send([self._inkjet_oid, 0, code])
        gcmd.respond_info("Nozzle code set to 0x%03X" % (code & 0xFFF))

    # ------------------------------------------------------------------
    # Physical button handlers (called from [gcode_button] gcode entries)
    # ------------------------------------------------------------------

    def _manual_move(self, axis, direction):
        """Issue a relative move on *axis* in *direction* (+1 / -1)."""
        dist  = direction * self.manual_step
        speed = self.manual_speed
        gcode = self.printer.lookup_object('gcode')
        if axis == 'Z':
            gcode.run_script_from_command(
                "SAVE_GCODE_STATE NAME=_inkjet_btn\n"
                "G91\n"
                "G1 Z%.4f F%d\n"
                "RESTORE_GCODE_STATE NAME=_inkjet_btn"
                % (dist, int(speed * 60)))
        elif axis == 'E1':
            # powder_feed is a [manual_stepper] – jog it directly
            gcode.run_script_from_command(
                "MANUAL_STEPPER STEPPER=powder_feed MOVE=%.4f SPEED=%.2f"
                % (dist, speed))

    def _evaluate_buttons(self):
        """Fire a manual move based on current button + selector state."""
        if not (self._btn_up or self._btn_down):
            return
        direction = 1 if self._btn_up else -1
        if self._btn_right:
            # 3-pos switch: RIGHT position → Z axis
            self._manual_move('Z', direction)
        elif self._btn_left:
            # 3-pos switch: LEFT position → E1 (powder feed)
            self._manual_move('E1', direction)
        # Centre (both selectors open): no action

    # Individual button callbacks ----------------------------------------

    def _btn_up_press(self, gcmd):
        self._btn_up = True
        self._evaluate_buttons()

    def _btn_up_release(self, gcmd):
        self._btn_up = False

    def _btn_down_press(self, gcmd):
        self._btn_down = True
        self._evaluate_buttons()

    def _btn_down_release(self, gcmd):
        self._btn_down = False

    def _btn_left_press(self, gcmd):
        self._btn_left = True

    def _btn_left_release(self, gcmd):
        self._btn_left = False

    def _btn_right_press(self, gcmd):
        self._btn_right = True

    def _btn_right_release(self, gcmd):
        self._btn_right = False

    # ------------------------------------------------------------------
    # Status query
    # ------------------------------------------------------------------

    def get_status(self, eventtime):
        code = self.nozzle_code
        return {
            'nozzle_code':     code,
            'nozzle_code_hex': '0x%03X' % (code & 0xFFF),
            'firing':          code < NOZZLE_CODE_IDLE,
        }


def load_config(config):
    return InkjetNozzle(config)
