# emission

Required: python 3

Run new_main_grid.py

Command line:
python new_main_grid.py krk <start_time> <interval_duration> <simualation_duration> <segment_size> <visualisation>
e.g. python new_main_grid.py krk 00:00 10s 20s 1000 false

Commad line with init file
python new_main_grid.py init_file init_file.json

init_file.json
{
  "init_data": {
	  "region":"krk",
	  "start_time":"00:00",
	  "interval_duration":"10s",
	  "simulation_duration":"30s",
	  "visualisation":"false",
	  "segment_size":"1000"
  }
}
