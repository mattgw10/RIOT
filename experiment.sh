python run.py -A RIOT -v dynamic_car -m ./maps/large_sparse_2D.txt -t 120 -n 50 -s 102 -g 9590
python run.py -A RIOT -v dynamic_car -m ./maps/house.txt -t 120 -n 50 -s 22 -g 268
python run.py -A RIOT -v hovercraft -m ./maps/large_sparse_2D.txt -t 120 -n 50 -s 102 -g 9590
python run.py -A RIOT -v hovercraft -m ./maps/house.txt -t 120 -n 50 -s 22 -g 268
python run.py -A RIOT -v generic_3D -m ./maps/slalom.txt -t 120 -n 50 -s 5005 -g 5905
python run.py -A RIOT -v generic_3D -m ./maps/maze_3D.txt -t 120 -n 50 -s 0 -g 999

python run.py -A RIOT -v dynamic_car -m ./maps/maze_2D.txt -t 120 -n 50 -s 102 -g 9590
python moving_ai_bench.py -A RIOT -v dynamic_car -t 120 -f ./maps/orz100d.map.scen
python moving_ai_bench.py -A RIOT -v dynamic_car -t 120 -f ./maps/Boston_0_256.map.scen
