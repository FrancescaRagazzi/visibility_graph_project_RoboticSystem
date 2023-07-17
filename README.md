<a name="readme-top"></a>

<br />
<div align="center">
  <a href="https://github.com/corradosantoro/RoboticSystems">
    <img src=".github/images/arslogo.png" alt="arslogo">
  </a>
<h3 align="center">RoboticSystems</h3>

  <p align="center">
    Software Simulation Environment for the Robotic Systems Course @ UniCT by <a href="https://github.com/corradosantoro">Prof. Santoro Corrado</a></p>
</div>

-------

<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#install">Install</a>
      <ul>
        <li><a href="#pythonic-way-for-linux-windows-macos">Pythonic Way for Linux-Windows-macOS</a></li>
        <li><a href="#windows-with-wsl">Windows with WSL</a></li>
      </ul>
    </li>
    <li>
      <a href="#usage">Usage</a>
    </li>
  </ol>
</details>

<br/>

## Install

The simulation environment can be installed in many ways, choose your preferred one.

First of all, you need to download this repository on your machine. You can download this repository as a zip file or use __git clone__ command:

      git clone https://github.com/corradosantoro/RoboticSystems

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Pythonic Way for Linux-Windows-macOS

We are going to create a __virtual environment__ to contains all the dependencies.

1. Install [Python 3](https://www.python.org/) on your system, at the time of writing it is recommended Python 3.9 or newer:
    
    - _Linux (Debian-based)_:
      ```bash
      sudo apt install python3 python3-pip
      ```
    - _Linux (RHEL-based)_:
      ```bash
      sudo dnf install python3 python3-pip
      ```
    - _Windows and macOS_:

          Use the proper installed provided on the official Python website
2. Create a virtual environment, open a terminal inside the `RoboticSystems` folder and execute the following command:

        python -m venv ./venv
3. Activate the virtual environment:

   - _Linux (bash shell) or macOS_:
     ```bash
     source ./venv/bin/activate
     ```
   - _Windows_ (cmd.exe):
     ```batch
     ./venv/Scripts/activate.bat
     ```
4. Install the required dependencies using `pip`:

       python -m pip install -r requirements.txt

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Windows with WSL

Depending on your Windows and WSL version you may need to install a __Xorg server__ on your machine.

#### Windows 11 and later

On Windows 11 you can install _WSL from Microsoft Store_ which contains WSLg, needed to run graphical applications on WSL.

1. Install `Windows Subsystem for Linux` from [here](https://apps.microsoft.com/store/detail/windows-subsystem-for-linux/9P9TQF7MRM4R)
2. Install a WSL Linux disto, [Ubuntu 22.04 LTS or later is recommended](https://apps.microsoft.com/store/detail/ubuntu-22042-lts/9PN20MSR04DW)
3. Follow the Pythonic Way steps for a Debian-based distro, you may need to clone this repository inside Linux filesystem

#### Windows 10 Build 19041 or later

On Windows 10 you can enable the embedded WSL system-binary, but you will need to install a _Xorg server_ to run graphical applications on WSL.

1. Enable WSL system-binary running the following command, be sure to run with administrator privileges:

         wsl --install
2. Install a WSL Linux disto, [Ubuntu 22.04 LTS or later is recommended](https://apps.microsoft.com/store/detail/ubuntu-22042-lts/9PN20MSR04DW)

3. Download and install <a target="_blank" href="https://sourceforge.net/projects/vcxsrv/">VcXsrv</a>. 
4. Open VcXsrv and follow these steps:

    <details>
    <summary>Show steps</summary>
    <div align="center">
    <img src="./.github/images/01.png" style="zoom: 67%;">

    <img src="./.github/images/02.png" style="zoom:67%;" >

    <img src="./.github/images/03.png" style="zoom:67%;" >

    <img src="./.github/images/04.png" style="zoom:64%;" >
    </div></details>

5. Open a terminal and execute:

    ```bash
    cat /etc/resolv.conf
    ```

    Take note of the nameserver.

    <img src="./.github/images/05.png">

6. Add environment variables on your bashrc profile:

    ```shell
    cd $HOME && nano .bashrc
    ```

    At the end insert:

    ```shell
    export DISPLAY=<nameserver>:0.0
    export LIBGL_ALWAYS_INDIRECT=1
    ```

    Use `CTRL+X` and then press `Y` to save and exit.

7. Logout and login from the current session or refresh bash profile running:

    ```shell
    source $HOME/.bashrc
    ```

8.  Follow the Pythonic Way steps for a Debian-based distro, you may need to clone this repository inside Linux filesystem

## Usage

Before running any script you need to activate the virtual environment containing the required dependencies:

   - _Linux (bash shell) or macOS_:
     ```bash
     source ./venv/bin/activate
     ```
   - _Windows_ (cmd.exe):
     ```batch
     ./venv/Scripts/activate.bat
     ```

You can now run any scripts of the simulation environment, example:

```bash
python ./tests/card_1d/test_card_gui.py
```
