'use strict';

var Backbone = require('backbone');
var ioClient = require('../ros-events.js');
var Dispatcher = require('../dispatcher');

/**
 * Model for the Agent.
 *
 * @param name String - name of the agent.
 * @param running Boolean - true if the agent is active.
 * @param state String - agent's current state.
 * @param topic String - socket.io topic to listen to for incoming data.
 * @param dispatcherTopic String - topic to notify the rest of the app.
 */

var Agent = Backbone.Model.extend({

  defaults: {
    name: 'Agent',
    running: false,
    state: 'OFF',
    strategy: 'Mission',
    dispatchertopic: 'agent:change',
    stateTopic: 'web/robot/state',
    victimAlertTopic: 'web/victim/alert',
    currentTargetTopic: 'web/agent/target',
    target: {
      id: '0',
      victimFrameId: '/map',
      position: {
        x: 0,
        y: 0,
        z: 9
      },
      orientation: {
        x: 0,
        y: 0,
        z: 0,
        w: 0
      },
      probability: 0,
      sensors: [
        'thermal',
        'co2'
      ]
    }
  },

  /**
   * Register events on the event dispatcher.'
   */

  initialize: function() {
    /**
     * Notify other parts of the app for the voltage change.
     */

    var _this = this;
    this.on('change', function() {
      Dispatcher.trigger(_this.get('dispatcherTopic'), _this.attributes);
    });

    this.on('change:alert', function() {
      console.log('alert arrived.');
      Dispatcher.trigger('agent:victim:alert', _this.attributes);
    });
    this.on('change:state', function() {
      Dispatcher.trigger('agent:change:state', _this.attributes);
    });
    this.on('change:mission', function() {
      Dispatcher.trigger('agent:change:mission', _this.attributes);
    });
    this.on('change:target', function() {
      Dispatcher.trigger('agent:change:target', _this.attributes);
    });
  },

  /**
   * Listen for incoming data from the socket.io server.
   */

  subscribe: function() {
    console.log('Listening on topic ' + this.get('stateTopic'));
    console.log('Listening on topic ' + this.get('victimAlertTopic'));
    console.log('Listening on topic ' + this.get('currentTargetTopic'));

    var _this = this;
    ioClient.on(this.get('stateTopic'), function(state) {
      _this.set({'state': state});
    });

    ioClient.on(this.get('victimAlertTopic'), function(msg) {
      console.log(msg);
      _this.set({'alert': msg});
    });

    ioClient.on(this.get('currentTargetTopic'), function(msg) {
      _this.set({'target': msg.target});
    });

  },

  unsubscribe: function() {
    ioClient.removeListener(this.get('target'));
    ioClient.removeListener(this.get('victimAlertTopic'));
    ioClient.removeListener(this.get('stateTopic'));
  }

});

module.exports = Agent;
