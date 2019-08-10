# Aura_VR-fira

## Compile and run:
`./compile.bash`
`./setup.bash`

### To run server, gazebo and world
`roslaunch aura world.launch`
#### From other worlds (world available in src/aura/world)
`roslaunch aura world.launch world:=world_name`
### To run client
#### Fast moving
`roslaunch aura start_fast.launch`
#### Slow moving (recommended)
`roslaunch aura start_slow.launch`
