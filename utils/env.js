'use strict';

/*
 * Configuration shared among the services.
 */

var env = {};

env.getRosMasterURI = function() {
  return process.env.ROS_MASTER_URI;
};

env.getRosMasterIP = function() {
  return "192.168.0.106";
};

env.rosBridgePort = 9090;

module.exports = env;
