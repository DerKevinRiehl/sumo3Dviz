Commands to reproduce the video script as working on MacOS:

# SOFTWARE SETUP OF SUMO3DVIZ

Create a virtual environment and activate it:

```bash
python3 -m venv venv
source venv/bin/activate # -> on Windows use correspondingly slightly different command
```

Install the required packages:

```bash
# Install package
pip install -e .

# For development (includes pytest)
pip install -e ".[dev]"
```

Run the video rendering script from the root of the repository:

```bash
python code/render_video.py
```

Dependencies: On MacOS it might be required to install Panda3D through the corresponding installer provided on their website (instead of only using the Python package - not sure, should be tested, current setup has both the python package and the installed version).

# PREPARE SIMULATION

You can run any SUMO simulation and render it to a video.
Just make sure to log vehicle positions and traffic lights (if desired for rendering).
Also, if you want to place trees, fences, buildings, and other objects, please create polygon files with netedit.
In the following explanations how to do it.
Moreover, we provide an example (barcelona_simulation) that demos all outlined information.

(1) **Log Vehicle Positions in your `Configuration.sumocfg`:**

```xml
<!-- YOUR Configuration.sumocfg -->
<?xml version="1.0" encoding="UTF-8"?>
<configuration xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/sumoConfiguration.xsd">

    <!-- ... -->

    <!-- INSERT THIS TO LOG VEHICLE POSITIONS -->
    <output>
        <fcd-output value="simulation_logs/vehicle_positions.xml"/>
        <fcd-output.attributes value="x,y,angle"/>
    </output>

    <!-- ... -->

</configuration>
```

(2) **Log Traffic Light States `Configuration.sumocfg`:**

```xml
<!-- YOUR Configuration.sumocfg -->
<?xml version="1.0" encoding="UTF-8"?>
<configuration xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/sumoConfiguration.xsd">

    <!-- ... -->

    <!-- INSERT THIS TO LOAD ADDITIONAL FILE tls_logging.add.xml -->
    <input>
		<additional-files value="tls_logging.add.xml"/>
    </input>

    <!-- ... -->

</configuration>
```

And create the additional file `tls_logging.add.xml` in the same folder:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<additional>
    <timedEvent type="SaveTLSStates"
                dest="simulation_logs/signal_states.xml"/>
</additional>
```

(3) **Additional Objects (Fences, Trees, Buildings...):**

You can create polygon files (POIs) with Netedit, and store them, for example following `trees.add.xml`:

```xml
<additional xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/additional_file.xsd">
    <!-- Shapes -->
    <poi id="poi_0" color="red" layer="202.00" x="19332.99" y="17853.26"/>
    <poi id="poi_1" color="red" layer="202.00" x="19398.22" y="17894.70"/>
    <poi id="poi_10" color="red" layer="202.00" x="19412.72" y="17919.65"/>
    <poi id="poi_100" color="red" layer="202.00" x="18935.17" y="17729.96"/>
    <poi id="poi_1000" color="red" layer="202.00" x="20139.72" y="18631.08"/>
    <poi id="poi_1001" color="red" layer="202.00" x="20154.28" y="18637.80"/>
    <poi id="poi_1002" color="red" layer="202.00" x="20205.22" y="18645.08"/>
    <poi id="poi_1003" color="red" layer="202.00" x="20209.14" y="18647.88"/>
    <!-- ... -->
```

This is then loaded by our software.

# Tutorial: sumo3Dviz Usage

Example with the barcelona network.

## Via Command Line

TODO

## Via Python Script

TODO
