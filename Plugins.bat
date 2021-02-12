:: install the NL_OPT package from the prebuild .whl file stored locally
pip install ./wheels/NLopt-2.7.0-cp37-cp37m-win_amd64.whl

:: install plotly extension
jupyter labextension install jupyterlab-plotly@4.14.3 --no-build

:: install ipywidgets extension (for interactivity)
jupyter labextension install @jupyter-widgets/jupyterlab-manager plotlywidget@4.14.3 --no-build

:: install debugger support
jupyter labextension install @jupyterlab/debugger --no-build

:: rebuild jupyterlab
jupyter lab build

:: list the installed extensions
jupyter labextension list