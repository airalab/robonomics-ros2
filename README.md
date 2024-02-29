<a name="readme-top"></a>

<!-- PROJECT SHIELDS -->
[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![Apache-2.0 License][license-shield]][license-url]


<!-- PROJECT TITLE -->
<br />
<div align="center">
<h3 align="center">Robonomics ROS 2 Wrapper</h3>

<p align="center">
    Python packages with simple wrapper of Robonomics parachain functions for Robot Operating System 2
</p>
</div>

<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About Project</a>
      <ul>
        <li><a href="#project-structure">Project Structure</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation-and-building">Installation and Building</a></li>
      </ul>
    </li>
    <li>
      <a href="#usage">Usage</a>
        <ul>
        <li><a href="#testing-with-turtlesim">Testing with Turtlesim</a></li>
        <li><a href="#programming-your-node">Programming Your Node</a></li>
        </ul>
    </li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>

<!-- ABOUT THE PROJECT -->
## About Project

This ROS 2 Python packages are dedicated to wrapping the [**Robonomics**](https://robonomics.network/) parachain API 
provided by [**robonomics-interface**](https://github.com/Multi-Agent-io/robonomics-interface) 
into nodes of ROS 2. Robonomics is a decentralized cloud for storing digital twins of robotics and 
IoT devices and control them through this network. 

The goal of the project is to provide ROS 2 developers with a convenient way to integrate their robots or devices 
with parachain features. The logic behind the integration of a robotic device is that a unique address is created 
for it in the Robonomics parachain, which is used to control the device or receive its telemetry.

Available features include: 

* **launch** — function of launching a device to execute any command with a specified set of parameters;
* **datalog** — function for publishing any device data.

To pack and unpack data, [InterPlanetary File System](https://ipfs.tech/) is used, which allows to access files 
by their unique hash.

To learn more about Robonomics, please refer to the official documentation: 
[wiki.robonomics.network](https://wiki.robonomics.network/).

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Project Structure

For convenience, the project is divided into several ROS 2 packages:

    .
    ├── ipfs_handler                            # An auxiliary package that implements ROS 2 services 
    │                                           # for uploading and downloading IPFS files
    ├── robonomics_ros2_examples                # Package with turtlesim example for testing
    ├── robonomics_ros2_interfaces              # A package that describes all types of ROS 2 services and messages
    ├── robonomics_ros2_pubsub                  # Main package for interaction with Robonomics
    │   ├── config
    │   │   ├── robonomics_params_template.yaml # Config file for account credentials, IPFS directory, etc.
    │   ├── robonomics_ros2_pubsub              
    │   │   ├── utils                           # Directory for various utility functions
    │   │   ├── robonomics_ros2_receiver.py     # ROS 2 node for receiving launch commands and datalog content 
    │   │   │                                   # from another account
    │   │   └── robonomics_ros2_sender.py       # ROS 2 node for sending launch commands and datalogs
    │   └── ...
    └── ...

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- GETTING STARTED -->
## Getting Started

### Prerequisites

Make sure you have the following software installed: 

* Linux OS distribution (tested on [Ubuntu 22.04.3](https://releases.ubuntu.com/jammy/))
* ROS 2 distribution (tested on [Humble version](https://docs.ros.org/en/humble/Installation.html))
* [Python 3](https://www.python.org/downloads/) (tested on 3.10.12)
* [IPFS node](https://docs.ipfs.tech/) (tested on [IPFS Kubo](https://docs.ipfs.tech/install/command-line/) 0.26.0)
* Project specific Python modules can be installed via:
    ```shell
    pip install -r requirements.txt
    ```

For testing:

* Turtlesim package for your ROS2 version. For Humble:
    ```shell
    sudo apt install ros-humble-turtlesim
    ```

You also need to create an account on Robonomics parachain and write down seed phrase and account type. Make sure, 
that you have a free account balance to make transactions. The guide is available here:
https://wiki.robonomics.network/docs/create-account-in-dapp

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Installation and Building

1. Create directory for ROS 2 workspace and the `src` subdirectory:
    ```shell
    mkdir -p your_project_ws/src
    cd your_project_ws/src
    ```

2. Clone this repo to `src` directory:
    ```shell
    git clone https://github.com/Fingerling42/robonomics-ros2.git
    ```
   
3. Copy and rename the configuration file template:
    ```shell
    cp robonomics_ros2_pubsub/config/robonomics_params_template.yaml robonomics_ros2_pubsub/config/robonomics_params.yaml
    ```
   
4. Insert the account seed phrase and the account type into new `robonomics_params.yaml` file:
    ```yaml
    /**:
      ros__parameters:
        seed: 'robot seed phrase'
        crypto_type: 'ED25519 or SR25519'
    ```
   
   > **WARNING**: The seed phrase is sensitive information that allows anyone to use your account. Make sure you don't 
   > upload a config file with it to GitHub or anywhere else.
   
   You may also want to change the directory where the files for IPFS will be stored. To do this, change the parameter
   `ipfs_files_path`.

5. Then you can test the repository with turtlesim package or make your own robot integration. Anyway, after that
you need to build the package. From `your_project_ws` directory run:
    ```shell
    colcon build
    ```
   
6. Source the package to the environment (you will need it to do in every console instance):
    ```shell
    source install/setup.bash
    ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- USAGE EXAMPLES -->
## Usage

### Testing with Turtlesim

1. First, launch IPFS Daemon:
    ```shell
    ipfs daemon
    ```

2. Run the ROS 2 launch file from example directory. It will launch all necessary nodes: turtlesim itself, Robonomics
integration for turtlesim and Robonomics ROS 2 wrapper:
    ```shell
    ros2 launch robonomics_ros2_examples turtlesim_robonomics_launch.py
    ```

3. You will see the simulator with turtle and ROS 2 logs in the console with IPFS ID, path to directory with IPFS files,
and Robonomics address. The node with Robonomics integration starts publish datalogs with a turtle position every 1 min. 
You can check the datalog transactions, using, for example, [Subscan](https://robonomics.subscan.io/) explorer (just 
enter your Robonomics address). 

4. The node also starts waiting for every launch command, that will be sent to specified address. You can test it using 
the [Robonomics parachain portal](https://polkadot.js.org/apps/?rpc=wss%3A%2F%2Fkusama.rpc.robonomics.network%2F#/extrinsics):
go to **Developers** → **Extrinsics** → **Submission** and find launch function. 

5. The turtle is controlled using `/cmd_vel` topic, so you need to prepare corresponding messages, that will go as
a launch parameter. For convenience, this messages need to be added as JSON-file to IPFS:
    ```json
    {
       "linear":{
          "x":[
             1.0,
             2.0
          ],
          "y":[
             0.0,
             0.0
          ],
          "z":[
             0.0,
             0.0
          ]
       },
       "angular":{
          "x":[
             0.0,
             0.0
          ],
          "y":[
             0.0,
             0.0
          ],
          "z":[
             1.0,
             2.0
          ]
       }
    }
    ```
    This JSON example will command the turtle to move twice.

6. Then, you will need to upload the JSON file to IPFS and get its CID in special format (32-byte hash). You can use the
following  Python script for this purpose, that will print the formatted string with CID in the console: 
    ```python
    import ipfs_api
    from robonomicsinterface.utils import ipfs_qm_hash_to_32_bytes
    
    cid = ipfs_api.publish('./turtle_param.json')
    
    launch_param = ipfs_qm_hash_to_32_bytes(cid)
    
    print(launch_param)
    ```
   
7. For sending launch on the Robonomics parachain portal just specified the turtlesim address and formatted string as 
the param. Submit transaction and watch for the simulation.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Programming Your Node

When programming your own robot, you will need to create an integration that will use all the wrapper services and topic.
> **NOTE**: The node should use `MultiThreadedExecutor()`, `MutuallyExclusiveCallbackGroup()` and `ReentrantCallbackGroup()` 
> to avoid deadlocks, when one callback function calls another callback function. Please, read more about this issue
> [here](https://docs.ros.org/en/humble/How-To-Guides/Using-callback-groups.html). It is recommended to use different 
> callback groups for launch and datalog processing:
```python
...
launch_callback_group = ReentrantCallbackGroup()
datalog_callback_group = MutuallyExclusiveCallbackGroup()
...
```
#### IPFS Services

For your node, you will need to create two service client, for uploading and downloading files:

```python
...
self.ipfs_upload_client = self.create_client(
    UploadToIPFS,
    'ipfs/upload',
    callback_group=datalog_callback_group,
)
...
def ipfs_upload_request(self, file_name):
    request = UploadToIPFS.Request()
    request.file_name = file_name
    future = self.ipfs_upload_client.call_async(request)
    self.executor.spin_until_future_complete(future)
    return future.result()
```

```python
...
self.ipfs_download_client = self.create_client(
    DownloadFromIPFS,
    'ipfs/download',
    callback_group=launch_callback_group,
)
...
def ipfs_download_request(self, cid, file_name):
    request = DownloadFromIPFS.Request()
    request.cid = cid
    request.file_name = file_name
    future = self.ipfs_download_client.call_async(request)
    self.executor.spin_until_future_complete(future)
...
```

#### Receiving Launch

A subscriber, that listens for appearing parameter for launch (usually its IPFS CID):

```python
...
self.subscriber_launch_param = self.create_subscription(
    String,
    'robonomics/launch_param',
    self.subscriber_launch_param_callback,
    10,
    callback_group=launch_callback_group,
)
self.subscriber_launch_param  # prevent unused variable warning
...
def subscriber_launch_param_callback(self, msg):
    cid = msg.data
    # Send request to IPFS service
    response = self.ipfs_download_request(cid, 'file_name_with_msgs')
    # Then you processing your messages
...
```

#### Sending Launch

A service client, that sends a parameter to specified address:

```python
...
self.send_launch_client = self.create_client(
    RobonomicsROS2SendLaunch,
    'robonomics/send_launch',
    callback_group=launch_callback_group,
)
...
def send_launch_request(self, address, param):
    request = RobonomicsROS2SendLaunch.Request()
    request.address = address
    request.param = param
    future = self.ipfs_upload_client.call_async(request)
    self.executor.spin_until_future_complete(future)
    return future.result()
...
```

#### Receiving Last Datalog

A service client, that receives a content of last datalog from specified address:

```python
...
self.receive_last_datalog_client = self.create_client(
    RobonomicsROS2ReceiveLastDatalog,
    'robonomics/receive_last_datalog',
    callback_group=datalog_callback_group,
)
...
def receive_last_datalog_request(self, address):
    request = RobonomicsROS2ReceiveLastDatalog.Request()
    request.address = address
    future = self.ipfs_upload_client.call_async(request)
    self.executor.spin_until_future_complete(future)
    return future.result()
...
```

#### Sending Datalog

A service client, that sends datalogs based on data from topic. It is recommended to use it with a Timer
class to control how often data is sent (sending too often can drain your account!):

```python
...
self.send_datalog_client = self.create_client(
    RobonomicsROS2SendDatalog,
    'robonomics/send_datalog',
    callback_group=datalog_callback_group,
)
...
def send_datalog_request(self, datalog_content):
    request = RobonomicsROS2SendDatalog.Request()
    request.datalog_content = datalog_content
    future = self.send_datalog_client.call_async(request)
    self.executor.spin_until_future_complete(future)
    return future.result()
...
self.datalog_timer = self.create_timer(
    60, # secs
    self.datalog_timer_callback,
)
...
def datalog_timer_callback(self):
   # Preparing IPFS file
   ...
   # Upload file to IPFS
   response_ipfs = self.ipfs_upload_request(self.file_name)
   ...
   # Sending datalog
   response_datalog = self.send_datalog_request(response_ipfs.cid)
...
```

#### File Encryption and Decryption

To protect your files before sending them to IPFS, you can encrypt them with your private key and give access only to 
specified public key. To do this, you need to specify the recipient and sender public addresses in the config.

```yaml
/robonomics_ros2_pubsub:
  ros__parameters:
    ...
    recipient_address: '' # An address that can open an encrypted file
    sender_address: ''    # An address from which the encrypted file can be opened by a robot
```

The following functions are available for encrypting and decrypting files:

```python
from robonomics_ros2_pubsub.utils.crypto_utils import encrypt_file, decrypt_file
...
file_crypt = encrypt_file(file_name, ipfs_dir)  # Creates new file with encrypted data
...
decrypt_file(file_name, ipfs_dir)               # Rewrites encrypted file with decrypted data
...
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- ROADMAP -->
## Roadmap

- [x] Add basic datalog and launch functions
- [x] Add IPFS support
- [x] Add file encryption
- [ ] Add checks for IPFS file availability
- [ ] Add support for RWS calls
- [ ] Add digital twin functionality
- [ ] Add a selection of the IPFS connection type
- [ ] Rosbag2 integration?

You can [open issue](https://github.com/Fingerling42/robonomics-ros2/issues) to request a function or ask for bug fix.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- CONTRIBUTING -->
## Contributing

If you have a suggestion or correction, please feel free to participate! You should:

1. Fork the project
2. Add and commit your changes
3. Open a pull request

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- LICENSE -->
## License

Distributed under the Apache-2.0 License. See `LICENSE.txt` for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- CONTACT -->
## Contact

Ivan Berman — [@berman_ivan](https://twitter.com/berman_ivan) — berman@robonomics.network

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- ACKNOWLEDGMENTS -->
## Acknowledgments

* [robonomics-interface docs](https://multi-agent-io.github.io/robonomics-interface/index.html)
* [Best-README-Template](https://github.com/othneildrew/Best-README-Template/)

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/Fingerling42/robonomics-ros2.svg?style=for-the-badge
[contributors-url]: https://github.com/Fingerling42/robonomics-ros2/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/Fingerling42/robonomics-ros2.svg?style=for-the-badge
[forks-url]: https://github.com/Fingerling42/robonomics-ros2/network/members
[stars-shield]: https://img.shields.io/github/stars/Fingerling42/robonomics-ros2.svg?style=for-the-badge
[stars-url]: https://github.com/Fingerling42/robonomics-ros2/stargazers
[issues-shield]: https://img.shields.io/github/issues/Fingerling42/robonomics-ros2.svg?style=for-the-badge
[issues-url]: https://github.com/Fingerling42/robonomics-ros2/issues
[license-shield]: https://img.shields.io/github/license/Fingerling42/robonomics-ros2.svg?style=for-the-badge
[license-url]: https://github.com/Fingerling42/robonomics-ros2/blob/master/LICENSE.txt