# Safe Human-Robot Collaboration with Franka Emika Panda 

## Prerequisites:

* Install pip3:
  ``` bash
  sudo apt install python3-pip
  ```

* Install sphinx:
  ``` bash
  python3 -m pip install sphinx
  ```

* To use ReadTheDocs, install the following sphinx extension:
  ``` bash
  python3 -m pip install sphinx_rtd_theme
  ```

* Install myst-parser:
  ``` bash
  python3 -m pip install myst-parser
  ```

* Install build-essential:
  ``` bash
  sudo apt-get install build-essential
  ```

* Install docutils
  ```bash
  python3 -m pip install docutils==0.16
  ```

## Visualize the documentation:

* Clone this repository in the folder of your choice:
  ``` bash
  git clone https://github.com/panda-brubotics/panda_constrained_control_tutorial.git
  ```
* Go to the panda_constrained_control_tutorial directory and build html: 
  ``` bash
  make html
  ```
* View the index of the tutorial:

  Navigate to the /build/html directory (not via the terminal), right-click on index.html, and open the index.html with your browser.

  You should now be able to see the tutorial in your web browser. 
* Everytime you change someting to the .rst files in the source folder and you want to see the changes on the web page, 
  you have to go to the panda_constrained_control_tutorial directory and build html: 
  ``` bash
  make html
  ```

