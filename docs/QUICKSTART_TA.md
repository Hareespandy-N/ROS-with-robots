# தமிழ் quick start

இது நாமே எழுதிய AMR simulation + research project. ROS install இல்லாமலேயே
Python 3.10 அல்லது அதற்கு மேல் இருந்தால் offline experiments ஓடும்.

```bash
git clone https://github.com/Hareespandy-N/ROS-with-robots.git
cd ROS-with-robots
python -m amr_research_lab benchmark --output results
python -m amr_research_lab dock
python -m unittest discover -s tests -v
```

`results` folder-ல் SVG pictures, CSV traces, JSON measurements வரும்.
Browser-ல் SVG open பண்ணி planned path, robot travelled path, speed plot
பார்க்கலாம். ஒவ்வொரு experiment-ன் status-ஐ JSON-ல் சரிபார்க்கவும்.

- `baseline`: fixed lookahead pure pursuit.
- `regulated`: clearance மற்றும் curvature பார்த்து speed குறைக்கும் variant.
- Temporary obstacle case-ல் robot simulation நிறுத்தி, obstacle மறைந்ததும் தொடரும்.
- Docking demo-ல் pose alignment மட்டும்; charger/PLC output கிடையாது.

ROS 2 Humble usage, RViz instructions [README](../README.md)-ல் இருக்கு.
இந்த project real robot-க்கு deploy செய்யப்படவில்லை. Simulation-ல் குறைந்த
error வந்தாலும் physical +/-1 mm accuracy **not proven yet**.

Code, diagrams, maps, plots எல்லாம் இந்த project-க்காக புதிதாக உருவாக்கப்பட்டவை.
Papers-ன் figures/PDF/code இங்கே copy பண்ணவில்லை; [source citations](RESEARCH.md)
சேர்த்திருக்கிறோம். நமது files MIT license-ல் உள்ளன; வெளிப்புற works-க்கு அவற்றின்
சொந்த rights/license பொருந்தும்.
