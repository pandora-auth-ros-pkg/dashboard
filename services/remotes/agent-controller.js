'use strict';

var zmq = require('zmq');
var psTree = require('ps-tree');
var responder = zmq.socket('rep');

var spawn = require('child_process').spawn;
var child;


/**
 * Kill the process tree of a spawned process.
 */

var killChild = function(pid, signal, callback) {

  signal = signal || 'SIGKILL';
  callback = callback || function() {};
  var killTree = true;

  if (killTree) {
    psTree(pid, function(err, children) {
      [pid].concat(
        children.map(function(p) {
          return p.PID;
        })
      ).forEach(function(tpid) {
        try {
          process.kill(tpid, signal);
        } catch (ex) {
          console.log(ex);
        }
      });
      callback();
    });
  } else {
    try {
      process.kill(pid, signal);
    } catch (ex) {
      console.log(ex);
      callback();
    }
  }
};

/**
 * Receiving requests.
 */

responder.on('message', function(request) {
  console.log('Received request: ' + request);
  if (request != 'stop') {
    if (child !== undefined) {
      console.log('Killing all running nodes.');
      killChild(child.pid, 'SIGKILL');
    }
    child = spawn('roslaunch',
            ['pandora_fsm', 'agent_standalone.launch', 'strategy:=' + request]);
    console.log('Waking up the agent.');

    child.stdout.on('data', function(data) {
      console.log(data.toString());
    });

    child.stderr.on('data', function(err) {
      if (err) {
        console.log(err.toString());
      }
    });

    child.stdin.on('data', function(data) {
      if (data) {
        console.log(data.toString());
      }
    });

    console.log('Process started with PID ' + child.pid);
    responder.send(child.pid);
  } else {
    if (child !== undefined) {
      console.log('Trying to kill the agent.');
      killChild(child.pid, 'SIGKILL', function() {
        console.log('Agent stopped.');
        responder.send(true);
      });
    } else {
      responder.send(false);
    }
  }
});


/**
 * Error handler.
 */

responder.bind('tcp://*:5555', function(err) {
  if (err) {
    console.log(err);
  } else {
    console.log('Listening on 5555...');
  }
});


/**
 * System event handler.
 */

process.on('SIGINT', function() {
  console.log('Shutting down.');
  responder.close();
});
