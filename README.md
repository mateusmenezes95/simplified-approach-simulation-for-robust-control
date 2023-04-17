# Automatica Paper Scripts
Repository to store the Matlab/Octave/Python scripts used to write the paper for the [Automatica 2022 congress]

[Automatica 2022 congress]: https://www.sba.org.br/cba2022/%o-evento

## Install

### Yalmip

```matlab
cd YALMIPfolderShouldbeHere
urlwrite('https://github.com/yalmip/yalmip/archive/master.zip','yalmip.zip');
unzip('yalmip.zip','yalmip')
addpath(genpath([pwd filesep 'yalmip']));
savepath
```