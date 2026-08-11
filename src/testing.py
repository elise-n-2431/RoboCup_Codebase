import tkinter as tk
from tkinter import ttk
from dataclasses import dataclass
from enum import Enum, auto
import time


# ============================================================
# STATES - Mirrors your C++ enums
# ============================================================

class NavState(Enum):
    STATIONARY = auto()
    ROAMING = auto()
    PURSUIT = auto()
    SORTING = auto()
    HOMING = auto()
    DROPPING = auto()
    COLLECTING = auto()


class CollectState(Enum):
    LOWERING_VERT = auto()
    VERT_REACHED = auto()
    LOWERING_HORI = auto()
    HORI_REACHED = auto()
    PICKING_UP = auto()
    RETURNING_SUCCESS = auto()
    RETURNING_FAILURE = auto()
    IDLE = auto()


# ============================================================
# STATE FLAGS
# ============================================================

@dataclass
class StateFlags:
    not_target_weight_onboard: bool = False
    target_identified: bool = False
    weight_in_entrance: bool = False
    dummy_identified: bool = False
    metal_identified: bool = False

    home_reached: bool = False
    dropoff_complete: bool = False

    collection_complete: bool = False
    collection_failed: bool = False

    timer_1: bool = False
    timer_2: bool = False
    timer_3: bool = False
    timer_4: bool = False

    weight_detected: bool = False
    no_vertical: bool = False
    no_horisontal: bool = False

    can_iterate: bool = False


# ============================================================
# STATE MACHINE
# ============================================================

class StateMachine:

    MAX_ITERATIONS = 3

    def __init__(self):
        self.flags = StateFlags()

        self.current_nav_state = NavState.STATIONARY
        self.prev_nav_state = NavState.STATIONARY

        self.current_collect_state = CollectState.LOWERING_VERT
        self.prev_collect_state = CollectState.LOWERING_VERT

        self.current_pickup_iterations = 0

        self.nav_state_entered_at = time.monotonic()
        self.collect_state_entered_at = time.monotonic()

        self.transition_log = []

    # --------------------------------------------------------
    # Utility
    # --------------------------------------------------------

    def set_flag(self, flag_name):
        setattr(self.flags, flag_name, True)
        self.log(f"FLAG RAISED: {flag_name}")

    def reset_flag(self, flag_name):
        setattr(self.flags, flag_name, False)

    def flag_is_set(self, flag_name):
        return getattr(self.flags, flag_name)

    def log(self, message):
        timestamp = time.strftime("%H:%M:%S")
        self.transition_log.append(f"[{timestamp}] {message}")

        # Keep log from growing forever
        if len(self.transition_log) > 100:
            self.transition_log.pop(0)

    # --------------------------------------------------------
    # Time
    # --------------------------------------------------------

    def time_in_nav_state(self):
        return (time.monotonic() - self.nav_state_entered_at) * 1000

    def time_in_collect_state(self):
        return (time.monotonic() - self.collect_state_entered_at) * 1000

    # --------------------------------------------------------
    # State changes
    # --------------------------------------------------------

    def change_nav_state(self, new_state, flag_name):

        if self.flag_is_set(flag_name):

            old_state = self.current_nav_state

            self.prev_nav_state = self.current_nav_state
            self.current_nav_state = new_state

            # Reset flag when consumed
            self.reset_flag(flag_name)

            self.nav_state_entered_at = time.monotonic()

            self.log(
                f"NAV: {old_state.name} -> {new_state.name}"
                f"    [consumed: {flag_name}]"
            )

            return True

        return False

    def change_collect_state(self, new_state, flag_name):

        if self.flag_is_set(flag_name):

            old_state = self.current_collect_state

            # Same logic as your C++ code
            if self.current_nav_state != NavState.COLLECTING:
                if new_state != CollectState.IDLE:
                    new_state = CollectState.IDLE

            self.prev_collect_state = self.current_collect_state
            self.current_collect_state = new_state

            # Reset flag when consumed
            self.reset_flag(flag_name)

            self.collect_state_entered_at = time.monotonic()

            self.log(
                f"COLLECT: {old_state.name} -> {new_state.name}"
                f"    [consumed: {flag_name}]"
            )

            return True

        return False

    # --------------------------------------------------------
    # Timer checking
    # --------------------------------------------------------

    def check_timers(self):
        """
        This is intentionally NOT an exact copy of your C++ timer
        implementation.

        Your C++ code currently has:

            if timer1:
            else if timer2:
            else if timer3:
            else if timer4:

        and all timers are 2000 ms.

        That means timer_1 prevents timer_2/3/4 from ever firing.

        For this test harness, timers are manually controlled using
        the GUI buttons.
        """
        pass

    # --------------------------------------------------------
    # Main state machine
    # --------------------------------------------------------

    def update(self):

        # ====================================================
        # NAVIGATION STATE MACHINE
        # ====================================================

        if self.current_nav_state == NavState.STATIONARY:

            self.change_nav_state(
                NavState.ROAMING,
                "not_target_weight_onboard"
            )

        elif self.current_nav_state == NavState.ROAMING:

            self.change_nav_state(
                NavState.PURSUIT,
                "target_identified"
            )

        elif self.current_nav_state == NavState.PURSUIT:

            self.change_nav_state(
                NavState.SORTING,
                "weight_in_entrance"
            )

        elif self.current_nav_state == NavState.SORTING:

            # Mirrors your two checks.
            #
            # If both flags are raised simultaneously, both may
            # be consumed during this update, so the second state
            # change wins.
            self.change_nav_state(
                NavState.ROAMING,
                "dummy_identified"
            )

            self.change_nav_state(
                NavState.COLLECTING,
                "metal_identified"
            )

        elif self.current_nav_state == NavState.HOMING:

            self.change_nav_state(
                NavState.DROPPING,
                "home_reached"
            )

        elif self.current_nav_state == NavState.DROPPING:

            self.change_nav_state(
                NavState.STATIONARY,
                "dropoff_complete"
            )

        # ====================================================
        # COLLECTION STATE MACHINE
        # ====================================================

        elif self.current_nav_state == NavState.COLLECTING:

            # Collection completed
            if self.flags.collection_complete:

                self.change_nav_state(
                    NavState.STATIONARY,
                    "collection_complete"
                )

                return

            # Collection failed
            if self.flags.collection_failed:

                self.change_nav_state(
                    NavState.ROAMING,
                    "collection_failed"
                )

                return

            # -----------------------------------------------
            # Collection substates
            # -----------------------------------------------

            if self.current_collect_state == CollectState.LOWERING_VERT:

                self.change_collect_state(
                    CollectState.VERT_REACHED,
                    "timer_1"
                )

            elif self.current_collect_state == CollectState.VERT_REACHED:

                self.change_collect_state(
                    CollectState.PICKING_UP,
                    "weight_detected"
                )

                self.change_collect_state(
                    CollectState.LOWERING_HORI,
                    "no_vertical"
                )

            elif self.current_collect_state == CollectState.LOWERING_HORI:

                self.change_collect_state(
                    CollectState.HORI_REACHED,
                    "timer_2"
                )

            elif self.current_collect_state == CollectState.HORI_REACHED:

                self.change_collect_state(
                    CollectState.PICKING_UP,
                    "weight_detected"
                )

                self.change_collect_state(
                    CollectState.RETURNING_FAILURE,
                    "no_horisontal"
                )

            elif self.current_collect_state == CollectState.PICKING_UP:

                self.change_collect_state(
                    CollectState.RETURNING_SUCCESS,
                    "timer_3"
                )

            elif self.current_collect_state == CollectState.RETURNING_SUCCESS:

                self.set_flag("collection_complete")

            elif self.current_collect_state == CollectState.RETURNING_FAILURE:

                self.change_collect_state(
                    CollectState.IDLE,
                    "timer_4"
                )

            elif self.current_collect_state == CollectState.IDLE:

                if self.current_pickup_iterations >= self.MAX_ITERATIONS:

                    self.set_flag("collection_failed")

                else:

                    self.change_collect_state(
                        CollectState.LOWERING_VERT,
                        "can_iterate"
                    )


# ============================================================
# GUI
# ============================================================

class StateMachineTester:

    def __init__(self, root):

        self.root = root
        self.root.title("State Machine Test Harness")
        self.root.geometry("1100x750")

        self.machine = StateMachine()

        self.build_gui()

        # Run state machine repeatedly
        self.update_gui()

    # --------------------------------------------------------
    # GUI construction
    # --------------------------------------------------------

    def build_gui(self):

        # ====================================================
        # TOP: CURRENT STATE
        # ====================================================

        state_frame = ttk.LabelFrame(
            self.root,
            text="Current State",
            padding=10
        )
        state_frame.pack(
            fill="x",
            padx=10,
            pady=10
        )

        self.nav_state_label = ttk.Label(
            state_frame,
            text="",
            font=("Arial", 20, "bold")
        )
        self.nav_state_label.grid(
            row=0,
            column=0,
            padx=20
        )

        self.collect_state_label = ttk.Label(
            state_frame,
            text="",
            font=("Arial", 20, "bold")
        )
        self.collect_state_label.grid(
            row=0,
            column=1,
            padx=20
        )

        self.prev_nav_label = ttk.Label(
            state_frame,
            text="",
            font=("Arial", 12)
        )
        self.prev_nav_label.grid(
            row=1,
            column=0
        )

        self.prev_collect_label = ttk.Label(
            state_frame,
            text="",
            font=("Arial", 12)
        )
        self.prev_collect_label.grid(
            row=1,
            column=1
        )

        self.iteration_label = ttk.Label(
            state_frame,
            text=""
        )
        self.iteration_label.grid(
            row=2,
            column=0,
            columnspan=2
        )

        # ====================================================
        # FLAG BUTTONS
        # ====================================================

        flag_frame = ttk.LabelFrame(
            self.root,
            text="Raise State Flag",
            padding=10
        )
        flag_frame.pack(
            fill="x",
            padx=10,
            pady=5
        )

        flags = [
            ("not_target_weight_onboard", "Not Target Weight"),
            ("target_identified", "Target Identified"),
            ("weight_in_entrance", "Weight In Entrance"),
            ("dummy_identified", "Dummy Identified"),
            ("metal_identified", "Metal Identified"),

            ("home_reached", "Home Reached"),
            ("dropoff_complete", "Dropoff Complete"),

            ("weight_detected", "Weight Detected"),
            ("no_vertical", "No Vertical"),
            ("no_horisontal", "No Horizontal"),

            ("can_iterate", "Can Iterate"),

            ("timer_1", "Timer 1"),
            ("timer_2", "Timer 2"),
            ("timer_3", "Timer 3"),
            ("timer_4", "Timer 4"),
        ]

        for index, (flag, text) in enumerate(flags):

            row = index // 4
            column = index % 4

            button = ttk.Button(
                flag_frame,
                text=text,
                command=lambda f=flag: self.raise_flag(f)
            )

            button.grid(
                row=row,
                column=column,
                padx=5,
                pady=5,
                sticky="ew"
            )

        for column in range(4):
            flag_frame.columnconfigure(
                column,
                weight=1
            )

        # ====================================================
        # COLLECTION CONTROL
        # ====================================================

        collection_frame = ttk.LabelFrame(
            self.root,
            text="Collection Controls",
            padding=10
        )
        collection_frame.pack(
            fill="x",
            padx=10,
            pady=5
        )

        ttk.Button(
            collection_frame,
            text="Increase Pickup Iteration",
            command=self.increment_iteration
        ).pack(
            side="left",
            padx=5
        )

        ttk.Button(
            collection_frame,
            text="Set Iterations = 3",
            command=self.set_max_iterations
        ).pack(
            side="left",
            padx=5
        )

        ttk.Button(
            collection_frame,
            text="Reset Machine",
            command=self.reset_machine
        ).pack(
            side="right",
            padx=5
        )

        # ====================================================
        # ACTIVE FLAGS
        # ====================================================

        active_frame = ttk.LabelFrame(
            self.root,
            text="Currently Raised Flags",
            padding=10
        )
        active_frame.pack(
            fill="x",
            padx=10,
            pady=5
        )

        self.active_flags_label = ttk.Label(
            active_frame,
            text="None",
            font=("Courier", 11)
        )
        self.active_flags_label.pack()

        # ====================================================
        # LOG
        # ====================================================

        log_frame = ttk.LabelFrame(
            self.root,
            text="Transition Log",
            padding=10
        )
        log_frame.pack(
            fill="both",
            expand=True,
            padx=10,
            pady=10
        )

        self.log_text = tk.Text(
            log_frame,
            height=15,
            state="disabled",
            font=("Courier", 10)
        )
        self.log_text.pack(
            fill="both",
            expand=True
        )

    # --------------------------------------------------------
    # Button actions
    # --------------------------------------------------------

    def raise_flag(self, flag_name):

        self.machine.set_flag(flag_name)

        # Immediately run an update so the transition happens
        # without waiting for the GUI refresh cycle.
        self.machine.update()

        self.refresh_display()

    def increment_iteration(self):

        self.machine.current_pickup_iterations += 1

        self.machine.log(
            f"Pickup iterations = "
            f"{self.machine.current_pickup_iterations}"
        )

        self.refresh_display()

    def set_max_iterations(self):

        self.machine.current_pickup_iterations = (
            self.machine.MAX_ITERATIONS
        )

        self.machine.log(
            f"Pickup iterations = "
            f"{self.machine.current_pickup_iterations}"
        )

        self.refresh_display()

    def reset_machine(self):

        self.machine = StateMachine()

        self.refresh_display()

    # --------------------------------------------------------
    # GUI updates
    # --------------------------------------------------------

    def update_gui(self):

        self.machine.update()

        self.refresh_display()

        # Run again in 50 ms
        self.root.after(
            50,
            self.update_gui
        )

    def refresh_display(self):

        # ----------------------------------------------------
        # Navigation state
        # ----------------------------------------------------

        nav = self.machine.current_nav_state

        self.nav_state_label.config(
            text=f"NAV: {nav.name}",
            foreground=self.nav_colour(nav)
        )

        self.prev_nav_label.config(
            text=(
                f"Previous NAV: "
                f"{self.machine.prev_nav_state.name}"
            )
        )

        # ----------------------------------------------------
        # Collection state
        # ----------------------------------------------------

        collect = self.machine.current_collect_state

        self.collect_state_label.config(
            text=f"COLLECT: {collect.name}",
            foreground="darkgreen"
        )

        self.prev_collect_label.config(
            text=(
                f"Previous COLLECT: "
                f"{self.machine.prev_collect_state.name}"
            )
        )

        # ----------------------------------------------------
        # Iterations
        # ----------------------------------------------------

        self.iteration_label.config(
            text=(
                f"Pickup iterations: "
                f"{self.machine.current_pickup_iterations} / "
                f"{self.machine.MAX_ITERATIONS}"
            )
        )

        # ----------------------------------------------------
        # Active flags
        # ----------------------------------------------------

        active = []

        for name, value in vars(self.machine.flags).items():

            if value:
                active.append(name)

        if active:
            self.active_flags_label.config(
                text=" | ".join(active),
                foreground="red"
            )
        else:
            self.active_flags_label.config(
                text="None",
                foreground="green"
            )

        # ----------------------------------------------------
        # Log
        # ----------------------------------------------------

        self.log_text.config(state="normal")
        self.log_text.delete("1.0", tk.END)

        for entry in self.machine.transition_log:
            self.log_text.insert(
                tk.END,
                entry + "\n"
            )

        self.log_text.see(tk.END)

        self.log_text.config(state="disabled")

    # --------------------------------------------------------
    # Colours
    # --------------------------------------------------------

    def nav_colour(self, state):

        colours = {
            NavState.STATIONARY: "gray",
            NavState.ROAMING: "blue",
            NavState.PURSUIT: "orange",
            NavState.SORTING: "purple",
            NavState.HOMING: "brown",
            NavState.DROPPING: "darkred",
            NavState.COLLECTING: "darkgreen",
        }

        return colours.get(
            state,
            "black"
        )


# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":

    root = tk.Tk()

    app = StateMachineTester(root)

    root.mainloop()
