PANDORA dashboard
---


An express/socket.io server is responsible for gathering data from the ROS
topics and transmitting them to all the connected clients.

The server is ROS independent and can run on any system that can ping the robot
you want to monitor.

#### Configuration

Set the robot's ip on `utils/env.js`.

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

and finally install [pm2](https://github.com/Unitech/PM2).


```
npm install -g pm2
```


#### Run the server

Start the express server with
```
npm start
```

or

```
pm2 start bin/www
```

and then start the services

```
pm2 start services.json
```

On the robot run the remote services found on `services/remotes`, but first
fill in the correct server's ip on remotes.json.

```
pm2 start remotes.json
```

Connect from your browser to `server_ip:3000`.

#### LICENSE

MIT
