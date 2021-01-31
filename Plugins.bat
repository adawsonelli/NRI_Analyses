
:: install plotly extension
jupyter labextension install jupyterlab-plotly@4.14.3 --no-build

::install ipywidgets extension (for interactivity)
jupyter labextension install @jupyter-widgets/jupyterlab-manager plotlywidget@4.14.3 --no-build

::install debugger support
jupyter labextension install @jupyterlab/debugger --no-build

::rebuild jupyterlab
jupyter lab build

::list the installed extensions
jupyter labextension list