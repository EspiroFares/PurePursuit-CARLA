# CARLA Pure Pursuit

> Personal side project built to explore high-speed vehicle control in CARLA.


![CARLA Simulation](Pictures/CarlaUE4.gif)

A lightweight Pure Pursuit controller built for the CARLA simulator — designed to stay sharp even when the vehicle mechanics get excited at high speeds.
This project focuses on smooth geometric path tracking with adaptive lookahead and a P-controlled throttle, allowing the vehicle to maintain stability and precision through fast turns and long straights.

## Installation & Setup

### Prerequisites

Before running the project, ensure you have the following installed:

- **Python** (Recommended version: 3.8–3.10)
- **CARLA Simulator** (version ≥ 0.9.13)
- **pip** (latest version recommended)

---

### Installation steps

1. **Clone the repository:**

```sh
git clone https://github.com/<your-username>/carla_pure_pursuit.git
cd carla_pure_pursuit
```

2.  **Create and activate a virtual environment:**
```sh
python3 -m venv venv
source venv/bin/activate
```

3. **To deactivate the virtual environment:**

```sh
deactivate
```

#### **Warning: From here on the virtual environment needs to be activated!**

If the virtual environment is deactivate or the terminal is restarted the
venv needs to be activated again!

4. **Upgrade/install pip:**

```sh
python3 -m pip install --upgrade pip
```

5. **Install required packages:**

```sh
python3 -m pip install -r requirements.txt --quiet --no-cache-dir
```

6. **Verify installation:**

```sh
python3 -c "import carla; print('CARLA module found!')"
```

### Running the demo

1. Start CARLA Simulator
```sh
./carlaUE4.sh
```
or on windows:

```sh
carlaUE4.exe
```

2. Run the Pure Pursuit demo:
```sh
python3 scripts/run_demo.py
```
The vehicle will spawn in the CARLA world and begin following a global route using the Pure Pursuit controller.

## Author

This project was developed as a **personal side project** alongside my university studies.  
It was built to explore how **Pure Pursuit control** can handle high-speed vehicle dynamics in the **CARLA simulator**,  
combining curiosity for autonomous systems with a passion for smooth and stable control.