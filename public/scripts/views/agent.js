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
var validationTempalte = require('../templates/validation-panel.hbs');

var victimDialog = require('../templates/victim.dialog.hbs');


var AgentView = Backbone.View.extend({

  template: agentTemplate,
  targetPanelTemplate: targetTemplate,
  statePanelTemplate: stateTemplate,
  signsOfLifeTemplate: signsOfLifeTemplate,
  validationPanelTemplate: validationTempalte,

  victimDialogTemplate: victimDialog,

  waitingForValidation: false,

  initialize: function() {
    console.log('View Agent initialized');
    var strategies = [
      {
        name: "Normal",
        value: "normal"
      },
      {
        name: "Exploration mapping",
        value: "mapping"
      }
    ];
    this.model.set({strategies: strategies});
    this.model.subscribe();
    Dispatcher.on('agent:change:state', this.updateState, this);
    Dispatcher.on('agent:change:target', this.updateTarget, this);
    Dispatcher.on('agent:victim:alert', this.showVictimAlert, this);
    Socket.on('web/signsOfLife', this.updateSignsOfLife.bind(this));
  },

  events: {
    'click #reject-victim': 'rejectVictim',
    'click #accept-victim': 'acceptVictim',
    'click #start-agent': 'startAgent',
    'click #stop-agent': 'stopAgent'
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

  updateSignsOfLife: function(msg) {
    var signsOfLife = {
      co2: {
        name: "CO2",
        value: msg.co2.toFixed(2),
        status: this.getSensorStatus(msg.co2)
      },
      sound: {
        name: "Sound",
        value: msg.sound.toFixed(2),
        status: this.getSensorStatus(msg.sound)
      },
      hazmat: {
        name: "Hazmat",
        value: msg.hazmat.toFixed(2),
        status: this.getSensorStatus(msg.hazmat)
      },
      motion: {
        name: "Motion",
        value: msg.motion.toFixed(2),
        status: this.getSensorStatus(msg.motion)
      },
      visual: {
        name: "Visual",
        value: msg.visualVictim.toFixed(2),
        status: this.getSensorStatus(msg.visualVictim)
      },
      thermal: {
        name: "Thermal",
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
      return "danger";
    } else {
      return "info";
    }
  },

  renderPartial: function(template, selector, debugMsg, model) {
    if (debugMsg !== undefined) console.log('Rendering ' + debugMsg);

    if (model !== undefined) {
      var context = template(model);
    } else {
      var context = template(this.model.toJSON());
    }
    this.$(selector).html(context);
  },

  render: function() {

    this.$el.html(this.template(this.model.toJSON()));

    this.renderPartial(this.statePanelTemplate, '#robot-state-panel', 'state panel');
    this.renderPartial(this.targetPanelTemplate, '#target-panel', 'target panel');
    this.renderPartial(this.signsOfLifeTemplate, '#signs-of-life-probabilities', 'signs-of-life panel');

    if (this.waitingForValidation === true) {
      this.renderPartial(
        this.validationPanelTemplate, '#validation-panel', 'validation panel');
    }

    return this;
  },

  showVictimAlert: function() {
    console.log('Show victim alert');
    this.waitingForValidation = true;

    this.render();
    this.model.set({validationImageTopic: '/kinect/rgb/image_raw'});

    // Show the modal.
    this.renderPartial(
      this.validationPanelTemplate, '#validation-panel', 'validation panel');

    // Show stacked notification.
    new PNotify({
      title: 'Victim arrived',
      text: "Go to agent's panel to validate it",
      hide: false,
      type: 'success',
      confirm: {
        confirm: true
      },
      buttons: {
        closer: false
      },
      history: {
        history: false
      }
    }).get().on('pnotify.confirm', function() {
      console.log('Moving to the validation panel.');
      window.location = '/#/agent';
    });

    return this;
  }

});


module.exports = AgentView;
