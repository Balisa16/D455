# D455-Sampling
Sampling code for D455 Camera

## Dependencies
### GLEW
```
sudo apt-get install -y libglew-dev
```
### Pangolin
```
https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build && cd build
cmake ..
make -j4
sudo make install
```
or use
```
sudo apt-get install libpango-1.0-0
```
</br>
Please recheck your pangolin installation with
```
echo $LD_LIBRARY_PATH
```
if in these path there is no ```libpango_core.so```, add it with 
```
export LD_LIBRARY_PATH=/path/to/directory:$LD_LIBRARY_PATH
```
in *~/.bashrc* and update ```source ~/.bashrc``` where */path/to/directory* is path of directory which contains libpango_core.so. Usually in */usr/local/lib*
