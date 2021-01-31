# NRI_Analyses
Analyses performed to answer design questions for NRI project

[![Binder](https://mybinder.org/badge_logo.svg)](https://mybinder.org/v2/gh/adawsonelli/NRI_Analyses/master)



# how to setup the project to run locally

1. download or clone repo locally
2. install miniconda and cd the the root directory
3. run the following command to setup the NRI conda enviornment

```batch
conda env create -f environment.yml
```
4. the enviornment is now setup except for the NL OPT library and Jupyterlab plugins.activate the NRI enviornment

```batch
conda activate NRI
```
5. then run the following command

```batch
Plugins.bat
```

6. you are now ready to develop with the optimal design library


