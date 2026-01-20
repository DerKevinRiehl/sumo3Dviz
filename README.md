Commands to reproduce the video script as working on MacOS:

Create a virtual environment and activate it:

```bash
python3 -m venv venv
source venv/bin/activate # -> on Windows use correspondingly slightly different command
```

Install the required packages:

```bash
pip install -r requirements.txt
```

Run the video rendering script from the root of the repository:

```bash
python code/render_video.py
```


Dependencies: On MacOS it might be required to install Panda3D through the corresponding installer provided on their website (instead of only using the Python package - not sure, should be tested, current setup has both the python package and the installed version).

