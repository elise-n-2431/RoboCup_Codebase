import tkinter as tk
from tkinter import ttk
import time


# ============================================================
# STATES
# ============================================================

NAV_STATES = [
    "STATIONARY",
    "ROAMING",
    "PURSUIT",
    "SORTING",
    "COLLECTING",
    "HOMING",
    "DROPPING"
]

COLLECT_STATES = [
    "LOWERING_VERT",
    "VERT_REACHED",
    "LOWERING_HORI",
    "HORI_REACHED",
    "PICKING_UP",
    "RETURNING_SUCCESS",
    "RETURNING_FAILURE",
    "IDLE"
]


# ============================================================
# TIMERS
# ============================================================

# Equivalent to:
#
# const unsigned long TIMER_1_DURATION = 5000;
#
# in your C++ code.
#
# Python uses seconds, so 5 seconds = 5.0
TIMER_1_DURATION = 5.0

TIMER_2_DURATION = 3.0
TIMER_3_DURATION = 2.0
TIMER_4_DURATION = 2.0


# ============================================================
# STATE
# ============================================================

current_nav_state = "STATIONARY"
prev_nav_state = "STATIONARY"

current_collect_state = "IDLE"
prev_collect_state = "IDLE"

# Equivalent to millis() timestamps
nav_state_entered_at = time.monotonic()
collect_state_entered_at = time.monotonic()


# ============================================================
# FLAGS
# ============================================================

STATE_FLAGS = {
    # Navigation
    "target_identified": False,
    "reverse_triggered": False,
    "reverse_complete": False,
    "home_reached": False,
    "collection_complete": False,
    "collection_failed": False,
    "dropoff_complete": False,
    "target_weight_onboard": False,
    "dummy_identified": False,
    "metal_identified": False,

    # Collection
    "weight_in_entrance": False,
    "weight_detected": False,
    "no_vertical": False,
    "no_horisontal": False,
    "dropping_complete": False,
    "returning_complete": False,
    "can_iterate": False,
    "cant_iterate": False,
}


# ============================================================
# TIMER FUNCTIONS
# ============================================================

def time_in_nav_state():
    return time.monotonic() - nav_state_entered_at


def time_in_collect_state():
    return time.monotonic() - collect_state_entered_at


def timer1_expired():
    return time_in_collect_state() >= TIMER_1_DURATION


def timer2_expired():
    return time_in_collect_state() >= TIMER_2_DURATION


def timer3_expired():
    return time_in_collect_state() >= TIMER_3_DURATION


def timer4_expired():
    return time_in_collect_state() >= TIMER_4_DURATION


# ============================================================
# STATE CHANGE FUNCTIONS
# ============================================================

def check_change_nav_state(new_state, flag):
    """
    Python equivalent of:

        checkChangeNavState(ROAMING, &STATE_FLAGS.target_weight_onboard);
    """

    global current_nav_state
    global prev_nav_state
    global nav_state_entered_at

    if not STATE_FLAGS[flag]:
        return

    if current_nav_state == new_state:
        return

    prev_nav_state = current_nav_state
    current_nav_state = new_state

    # Equivalent to:
    #
    # navStateEnteredAt = millis();

    nav_state_entered_at = time.monotonic()

    log_transition(
        f"NAV: {prev_nav_state} -> {current_nav_state}"
        f"  [flag: {flag}]"
    )

    # Consume the flag
    STATE_FLAGS[flag] = False


def check_change_collect_state(new_state, condition):
    """
    Python equivalent of:

        checkChangeCollectState(
            VERT_REACHED,
            timer1Expired()
        );

    or:

        checkChangeCollectState(
            PICKING_UP,
            STATE_FLAGS.weight_detected
        );
    """

    global current_collect_state
    global prev_collect_state
    global collect_state_entered_at

    if not condition:
        return

    if current_collect_state == new_state:
        return

    # Your C++ logic:
    #
    # if (current_nav_state != STATIONARY) {
    #     if (collectState != IDLE) {
    #         collectState = IDLE;
    #     }
    # }
    #
    # This simulator follows that intention.

    actual_state = new_state

    if current_nav_state != "STATIONARY":
        if new_state != "IDLE":
            actual_state = "IDLE"

    prev_collect_state = current_collect_state
    current_collect_state = actual_state

    # Equivalent to:
    #
    # collectStateEnteredAt = millis();

    collect_state_entered_at = time.monotonic()

    log_transition(
        f"COLLECT: {prev_collect_state} -> {current_collect_state}"
    )


# ============================================================
# STATE MACHINE
# ============================================================

def update_state_machine():

    # --------------------------------------------------------
    # NAVIGATION STATE MACHINE
    # --------------------------------------------------------

    if current_nav_state == "STATIONARY":

        check_change_nav_state(
            "ROAMING",
            "target_weight_onboard"
        )

    elif current_nav_state == "ROAMING":

        check_change_nav_state(
            "PURSUIT",
            "target_identified"
        )

    elif current_nav_state == "PURSUIT":

        check_change_nav_state(
            "SORTING",
            "weight_in_entrance"
        )

    elif current_nav_state == "SORTING":

        # Important:
        # Your C++ currently checks these independently.

        check_change_nav_state(
            "ROAMING",
            "dummy_identified"
        )

        check_change_nav_state(
            "COLLECTING",
            "metal_identified"
        )

    elif current_nav_state == "HOMING":

        check_change_nav_state(
            "DROPPING",
            "home_reached"
        )

    elif current_nav_state == "DROPPING":

        check_change_nav_state(
            "STATIONARY",
            "dropoff_complete"
        )

    elif current_nav_state == "COLLECTING":

        # Navigation transitions out of collection

        check_change_nav_state(
            "STATIONARY",
            "collection_complete"
        )

        check_change_nav_state(
            "ROAMING",
            "collection_failed"
        )

        # ----------------------------------------------------
        # COLLECTION STATE MACHINE
        # ----------------------------------------------------

        if current_collect_state == "LOWERING_VERT":

            check_change_collect_state(
                "VERT_REACHED",
                timer1_expired()
            )

        elif current_collect_state == "VERT_REACHED":

            check_change_collect_state(
                "PICKING_UP",
                STATE_FLAGS["weight_detected"]
            )

            check_change_collect_state(
                "LOWERING_HORI",
                STATE_FLAGS["no_vertical"]
            )

        elif current_collect_state == "LOWERING_HORI":

            check_change_collect_state(
                "HORI_REACHED",
                timer2_expired()
            )

        elif current_collect_state == "HORI_REACHED":

            check_change_collect_state(
                "PICKING_UP",
                STATE_FLAGS["weight_detected"]
            )

            check_change_collect_state(
                "RETURNING_FAILURE",
                STATE_FLAGS["no_horisontal"]
            )

        elif current_collect_state == "PICKING_UP":

            check_change_collect_state(
                "RETURNING_SUCCESS",
                timer3_expired()
            )

        elif current_collect_state == "RETURNING_SUCCESS":

            # Add your success logic here
            pass

        elif current_collect_state == "RETURNING_FAILURE":

            check_change_collect_state(
                "IDLE",
                timer4_expired()
            )

        elif current_collect_state == "IDLE":

            check_change_collect_state(
                "LOWERING_VERT",
                STATE_FLAGS["can_iterate"]
            )


# ============================================================
# LOG
# ============================================================

transition_log = []


def log_transition(message):

    timestamp = time.strftime("%H:%M:%S")

    transition_log.append(
        f"{timestamp}   {message}"
    )

    # Keep the log manageable
    if len(transition_log) > 100:
        transition_log.pop(0)


# ============================================================
# GUI
# ============================================================

root = tk.Tk()
root.title("State Machine Simulator")
root.geometry("1150x750")


# ------------------------------------------------------------
# Colours
# ------------------------------------------------------------

GREEN = "#2ecc71"
RED = "#e74c3c"
BLUE = "#3498db"
DARK = "#2c3e50"
GREY = "#ecf0f1"
YELLOW = "#f1c40f"


# ------------------------------------------------------------
# Top state display
# ------------------------------------------------------------

state_frame = ttk.LabelFrame(
    root,
    text="Current State",
    padding=15
)

state_frame.pack(
    fill="x",
    padx=10,
    pady=10
)


nav_label = tk.Label(
    state_frame,
    text="NAV: STATIONARY",
    font=("Arial", 22, "bold"),
    fg=BLUE
)

nav_label.grid(
    row=0,
    column=0,
    padx=30
)


collect_label = tk.Label(
    state_frame,
    text="COLLECT: IDLE",
    font=("Arial", 22, "bold"),
    fg=GREEN
)

collect_label.grid(
    row=0,
    column=1,
    padx=30
)


prev_nav_label = tk.Label(
    state_frame,
    text="Previous NAV: STATIONARY",
    font=("Arial", 11)
)

prev_nav_label.grid(
    row=1,
    column=0,
    padx=30,
    pady=5
)


prev_collect_label = tk.Label(
    state_frame,
    text="Previous COLLECT: IDLE",
    font=("Arial", 11)
)

prev_collect_label.grid(
    row=1,
    column=1,
    padx=30,
    pady=5
)


# ------------------------------------------------------------
# Timer display
# ------------------------------------------------------------

timer_frame = ttk.LabelFrame(
    root,
    text="Timers",
    padding=10
)

timer_frame.pack(
    fill="x",
    padx=10,
    pady=5
)


collect_timer_label = tk.Label(
    timer_frame,
    text="Time in collect state: 0.00 s",
    font=("Arial", 14)
)

collect_timer_label.pack(
    side="left",
    padx=20
)


timer1_label = tk.Label(
    timer_frame,
    text=f"Timer 1: 0.00 / {TIMER_1_DURATION:.2f} s",
    font=("Arial", 14)
)

timer1_label.pack(
    side="left",
    padx=20
)


timer1_status_label = tk.Label(
    timer_frame,
    text="NOT EXPIRED",
    font=("Arial", 14, "bold"),
    fg=GREEN
)

timer1_status_label.pack(
    side="left",
    padx=20
)


# ------------------------------------------------------------
# Flags
# ------------------------------------------------------------

flags_frame = ttk.LabelFrame(
    root,
    text="STATE_FLAGS — click to toggle",
    padding=10
)

flags_frame.pack(
    side="left",
    fill="y",
    padx=10,
    pady=10
)


flag_buttons = {}


def toggle_flag(flag):

    STATE_FLAGS[flag] = not STATE_FLAGS[flag]

    update_flag_button(flag)


def update_flag_button(flag):

    button = flag_buttons[flag]

    if STATE_FLAGS[flag]:

        button.config(
            text=f"ON   {flag}",
            bg=GREEN,
            activebackground=GREEN
        )

    else:

        button.config(
            text=f"OFF  {flag}",
            bg=RED,
            activebackground=RED
        )


def create_flag_button(parent, flag, row):

    button = tk.Button(
        parent,
        text=f"OFF  {flag}",
        width=27,
        bg=RED,
        fg="white",
        font=("Arial", 10, "bold"),
        command=lambda: toggle_flag(flag)
    )

    button.grid(
        row=row,
        column=0,
        pady=2
    )

    flag_buttons[flag] = button


for row, flag in enumerate(STATE_FLAGS.keys()):

    create_flag_button(
        flags_frame,
        flag,
        row
    )


# ------------------------------------------------------------
# Right side
# ------------------------------------------------------------

right_frame = tk.Frame(root)

right_frame.pack(
    side="right",
    fill="both",
    expand=True,
    padx=10,
    pady=10
)


# ------------------------------------------------------------
# Transition log
# ------------------------------------------------------------

log_frame = ttk.LabelFrame(
    right_frame,
    text="Transition Log",
    padding=10
)

log_frame.pack(
    fill="both",
    expand=True
)


log_text = tk.Text(
    log_frame,
    height=20,
    width=70,
    font=("Courier", 10),
    state="disabled"
)

log_text.pack(
    fill="both",
    expand=True
)


def update_log():

    log_text.config(state="normal")

    log_text.delete(
        "1.0",
        tk.END
    )

    for entry in transition_log:

        log_text.insert(
            tk.END,
            entry + "\n"
        )

    log_text.see(tk.END)

    log_text.config(state="disabled")


# ------------------------------------------------------------
# Controls
# ------------------------------------------------------------

controls_frame = ttk.LabelFrame(
    right_frame,
    text="Controls",
    padding=10
)

controls_frame.pack(
    fill="x",
    pady=10
)


def reset_machine():

    global current_nav_state
    global prev_nav_state
    global current_collect_state
    global prev_collect_state
    global nav_state_entered_at
    global collect_state_entered_at

    current_nav_state = "STATIONARY"
    prev_nav_state = "STATIONARY"

    current_collect_state = "IDLE"
    prev_collect_state = "IDLE"

    nav_state_entered_at = time.monotonic()
    collect_state_entered_at = time.monotonic()

    for flag in STATE_FLAGS:

        STATE_FLAGS[flag] = False
        update_flag_button(flag)

    transition_log.clear()

    log_transition("SYSTEM RESET")


reset_button = tk.Button(
    controls_frame,
    text="RESET STATE MACHINE",
    bg=DARK,
    fg="white",
    font=("Arial", 11, "bold"),
    command=reset_machine
)

reset_button.pack(
    side="left",
    padx=10
)


def clear_log():

    transition_log.clear()


clear_button = tk.Button(
    controls_frame,
    text="CLEAR LOG",
    command=clear_log
)

clear_button.pack(
    side="left",
    padx=10
)


# ============================================================
# GUI UPDATE LOOP
# ============================================================

def update_gui():

    # --------------------------------------------------------
    # Run state machine
    # --------------------------------------------------------

    update_state_machine()

    # --------------------------------------------------------
    # State labels
    # --------------------------------------------------------

    nav_label.config(
        text=f"NAV: {current_nav_state}"
    )

    collect_label.config(
        text=f"COLLECT: {current_collect_state}"
    )

    prev_nav_label.config(
        text=f"Previous NAV: {prev_nav_state}"
    )

    prev_collect_label.config(
        text=f"Previous COLLECT: {prev_collect_state}"
    )

    # --------------------------------------------------------
    # Timer
    # --------------------------------------------------------

    elapsed = time_in_collect_state()

    collect_timer_label.config(
        text=f"Time in collect state: {elapsed:.2f} s"
    )

    timer1_label.config(
        text=(
            f"Timer 1: "
            f"{elapsed:.2f} / "
            f"{TIMER_1_DURATION:.2f} s"
        )
    )

    if timer1_expired():

        timer1_status_label.config(
            text="EXPIRED",
            fg=RED
        )

    else:

        timer1_status_label.config(
            text="NOT EXPIRED",
            fg=GREEN
        )

    # --------------------------------------------------------
    # Log
    # --------------------------------------------------------

    update_log()

    # --------------------------------------------------------
    # Run again in 50 ms
    # --------------------------------------------------------

    root.after(
        50,
        update_gui
    )


# ============================================================
# START
# ============================================================

log_transition("SYSTEM STARTED")

update_gui()

root.mainloop()