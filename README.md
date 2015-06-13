PANDORA dashboard
---


An express/socket.io server is responsible for gathering data from the ROS
topics and transmitting them to all the connected clients.

The server is ROS independent and can run on any system that can ping the robot
you want to monitor.

#### Configuration

Just export the robot's ip to the `ROS_MASTER_URI`

```bash
export ROS_MASTER_URI=http://192.168.1.100:11311
```

On the robot run the [rosbridge](http://wiki.ros.org/rosbridge_suite) and the [web_video_server](https://github.com/RobotWebTools/web_video_server) for the image
streaming.

#### Installation

Fetch the dependencies

```
npm install
```

and build the app

```
npm install -g gulp
gulp build
```

#### Run the server

Start the express server with
```
npm start
````

and then start the services. Preferably with [pm2](https://github.com/Unitech/PM2).

Connect from your browser (desktop, mobile) to port `3000`.

#### LICENSE

MIT
