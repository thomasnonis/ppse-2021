# Abstract
This is the repository for SALMO, the Solar Azimuth and Longitude Motorized lOcator. It is a PCB used for driving a tracking solar panel system, using GPS location and a MPPT algorithm to maximise the incident power.  
The official name of the project is `PPS2021_SUN_TRK`

# Repo structure & guidelines

## Commits

A good commit message should be preceded by a prefix, then a colon and then a brief and descriptive comment on the changes made.  
All commits should be written in the present simple tense (eg. add file, modify this, edit that).  
Commits may also include longer descriptions in the second argument of a commit message.

Following are the prefix conventions for this repository.

- `hw:` for hardware (schematic, pcb, ...)
- `fw:` for firmware
- `sw:` for software (interfaces on a pc and such)
- `docs:` for documentation
- `notes:` for lecture notes
- `chore:` for general tasks (file management, moving stuff around, ...)

:x: `git commit -m "Added some features to code"`  
:heavy_check_mark: `git commit -m "fw: add uart implementation to gps driver"`

## File naming

All files must have no spaces and should be lowercase.

:x: Name of file.ext  
:heavy_check_mark: name-of-file.ext 