'use strict';

var Backbone = require('backbone');
var Dispatcher = require('../dispatcher');
var Socket = require('../ros-events');

/**
 * View template.
 */

var agentTemplate = require('../templates/agent.hbs');

var targetTemplate = require('../templates/target.panel.hbs');
var stateTemplate = require('../templates/state.panel.hbs');
var signsOfLifeTemplate = require('../templates/signs_of_life.panel.hbs');


var AgentView = Backbone.View.extend({

  template: agentTemplate,
  targetPanelTemplate: targetTemplate,
  statePanelTemplate: stateTemplate,
  signsOfLifeTemplate: signsOfLifeTemplate,

  waitingForValidation: false,
  targetID: 0,

  initialize: function() {
    console.log('View Agent initialized');
    var strategies = [
      {
        name: 'Normal',
        value: 'normal'
      },
      {
        name: 'Exploration mapping',
        value: 'mapping'
      }
    ];

    var modes = [{
        name: 'Off',
        value: 0
      }, {
        name: 'Autonomous',
        value: 1
      }, {
        name: 'Exploration rescue',
        value: 2
      }, {
        name: 'Identification',
        value: 3
      }, {
        name: 'Sensor hold',
        value: 4
      }, {
        name: 'Semi autonomous',
        value: 5
      }, {
        name: 'Teleoperation',
        value: 6
      }, {
        name: 'Sensor Test',
        value: 7
      }, {
        name: 'Exploration mapping',
        value: 8
      }, {
        name: 'Terminating',
        value: 9
      }];
    this.model.set({modes: modes});
    this.model.set({strategies: strategies});
    this.model.subscribe();
    Dispatcher.on('agent:change:state', this.updateState, this);
    Dispatcher.on('agent:change:target', this.updateTarget, this);

    Socket.on('web/signsOfLife', this.updateSignsOfLife.bind(this));
    Socket.on('web/agent/status/success', this.showAgentSuccessAlert.bind(this));
    Socket.on('web/agent/status/error', this.showAgentErrorAlert.bind(this));
    Socket.on('web/agent/status/pid', this.showAgentPIDAlert.bind(this));
    Socket.on('web/victimProbabilities/error', this.showVictimInfoError.bind(this));
    Socket.on('web/victimProbabilities/response', this.showVictimInfo.bind(this));
  },

  events: {
    'click #reject-victim': 'rejectVictim',
    'click #accept-victim': 'acceptVictim',
    'click #start-agent': 'startAgent',
    'click #stop-agent': 'stopAgent',
    'click #kill-agent': 'killAgent',
    'click #change-robot-mode': 'changeRobotMode',
    'click #update-victim-info': 'updateVictimInfo'
  },

  showVictimInfoError: function() {
    console.log('Error fetching probabilities.');

    new PNotify({
      title: 'Victim #' + this.targetID,
      text: "Couldn't retrieve probabilities.",
      type: 'error'
    });
  },

  showVictimInfo: function(data) {
    console.log(data);
    var model = {
      probabilities: data,
      id: this.targetID
    };
    this.renderPartial(this.targetPanelTemplate, '#target-panel', 'target panel', model);
  },

  updateVictimInfo: function() {
    console.log('Updating victim info.');

    this.targetID = $("#target-victim-id").val();

    // Send the request.
    Socket.emit('web/victimProbabilities/get', this.targetID);

    console.log(this.targetID);
  },

  changeRobotMode: function(event) {
    var mode = $('#robot-mode-selection').val();
    console.log(mode);
    Socket.emit('web/robot/changeMode', mode);
  },

  rejectVictim: function() {
    console.log('Victim rejected');
    this.waitingForValidation = false;
    Socket.emit('web/victim/response', false);
    this.$('#validation-panel').addClass('.hide').fadeOut(1000);
  },

  acceptVictim: function() {
    console.log('Victim accepted');
    this.waitingForValidation = false;
    Socket.emit('web/victim/response', true);
    this.$('#validation-panel').addClass('.hide').fadeOut(1000);
  },

  startAgent: function() {
    var strategy = $('#strategy-selection').val();
    console.log('Sending ' + strategy + ' command to the robot agent.');
    Socket.emit('web/agent/command', strategy);
  },

  stopAgent: function() {
    console.log('Sending stop command to the robot agent.');
    Socket.emit('web/agent/command', 'stop');
  },

  killAgent: function() {
    console.log('Sending kill command to the robot agent.');
    Socket.emit('web/agent/command', 'kill');
  },

  updateSignsOfLife: function(msg) {
    var signsOfLife = {
      co2: {
        name: 'CO2',
        value: msg.co2.toFixed(2),
        status: this.getSensorStatus(msg.co2)
      },
      sound: {
        name: 'Sound',
        value: msg.sound.toFixed(2),
        status: this.getSensorStatus(msg.sound)
      },
      hazmat: {
        name: 'Hazmat',
        value: msg.hazmat.toFixed(2),
        status: this.getSensorStatus(msg.hazmat)
      },
      motion: {
        name: 'Motion',
        value: msg.motion.toFixed(2),
        status: this.getSensorStatus(msg.motion)
      },
      visual: {
        name: 'Visual',
        value: msg.visualVictim.toFixed(2),
        status: this.getSensorStatus(msg.visualVictim)
      },
      thermal: {
        name: 'Thermal',
        value: msg.thermal.toFixed(2),
        status: this.getSensorStatus(msg.thermal)
      }
    };
    msg = {
      signsOfLife: signsOfLife
    };
    this.renderPartial(
      this.signsOfLifeTemplate,
      '#signs-of-life-probabilities', 'signs-of-life panel', msg);
  },

  updateState: function() {
    console.log(this.model.get('state'));
    this.renderPartial(
      this.statePanelTemplate, '#robot-state-panel', 'state panel');
  },

  updateTarget: function() {
    this.renderPartial(
      this.targetPanelTemplate, '#target-panel', 'target panel');
  },

  getSensorStatus: function(value) {
    if (value > 60) {
      return 'danger';
    } else {
      return 'info';
    }
  },

  renderPartial: function(template, selector, debugMsg, model) {
    var context;
    if (debugMsg !== undefined) console.log('Rendering ' + debugMsg);

    if (model !== undefined) {
      context = template(model);
    } else {
      context = template(this.model.toJSON());
    }
    this.$(selector).html(context);
  },

  render: function() {

    this.$el.html(this.template(this.model.toJSON()));

    this.renderPartial(this.statePanelTemplate, '#robot-state-panel', 'state panel');
    this.renderPartial(this.targetPanelTemplate, '#target-panel', 'target panel');
    this.renderPartial(this.signsOfLifeTemplate, '#signs-of-life-probabilities', 'signs-of-life panel');

    return this;
  },

  showAgentSuccessAlert: function() {
    console.log('Agent stopped');

    new PNotify({
      title: 'Agent terminated successfully',
      text: '',
      type: 'success'
    });
  },

  showAgentErrorAlert: function() {
    console.log('Agent error.');

    new PNotify({
      title: 'Agent Error',
      text: 'The agent didnt exit gracefully.',
      type: 'error'
    });
  },

  showAgentPIDAlert: function(pid) {
    console.log('Agent started with pid: ' + pid);

    new PNotify({
      title: 'Agent started.',
      text: 'PID: ' + pid,
      type: 'info'
    });
  }

});


module.exports = AgentView;
