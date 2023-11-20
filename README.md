## MapMatching
This project is a fork of git@github.com:laozheng1/MapMatching.git
used to study map matching and to have blank page syndrome

# Prepare env
```bash
conda create -n mapmatching  python=3.11
conda activate mapmatching
pip install -r requirements.txt
```
Simple output
```bash
mkdir result
python src/Mapmatching.py --maps data/map_information.csv --gpsdata data/gps_track.dat --output result/output.txt
```

output with plotting
```bash
mkdir result
python src/Mapmatching.py --maps data/map_information.csv --gpsdata data/gps_track.dat --output result/output.txt
```


The output will be stored in the "output.txt" file within the "result" directory. Additionally, the visualization results of the roads will be output in the "pic" folder located in the same directory as "output.txt."

# Dateset
gps_track.dat

Each piece of data represents a GPS coordinate and orientation of a specific car. In each data row, from left to right, the fields are: car number, time, longitude, latitude, speed, and direction.

File "map_information.csv" is used to represent map information. 
Each piece of data in this file contains information about a road on the map.