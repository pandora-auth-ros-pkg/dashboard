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
var missionTemplate = require('../templates/mission.panel.hbs');
var validationTempalte = require('../templates/validation-panel.hbs');

var victimNotification = require('../templates/victim.alert.hbs');
var victimDialog = require('../templates/victim.dialog.hbs');


var AgentView = Backbone.View.extend({

  template: agentTemplate,
  targetPanelTemplate: targetTemplate,
  statePanelTemplate: stateTemplate,
  missionPanelTemplate: missionTemplate,
  validationPanelTemplate: validationTempalte,

  victimNotificationTemplate: victimNotification,
  victimDialogTemplate: victimDialog,

  waitingForValidation: false,

  initialize: function() {
    console.log('View Agent initialized');

    this.model.subscribe();
    Dispatcher.on('agent:change:state', this.udpateState, this);
    Dispatcher.on('agent:change:target', this.updateTarget, this);
    Dispatcher.on('agent:change:mission', this.updateMission, this);
    Dispatcher.on('agent:victim:alert', this.showVictimAlert, this);
  },

  events: {
    'click #reject-victim': 'rejectVictim',
    'click #accept-victim': 'acceptVictim'
  },

  rejectVictim: function() {
    console.log('Victim rejected');
    this.waitingForValidaiton = false;
    Socket.emit('web/victim/response', false);
    this.$('#validation-panel').addClass('.hide').fadeOut(1000);
  },

  acceptVictim: function() {
    console.log('Victim accepted');
    this.waitingForValidaiton = false;
    Socket.emit('web/victim/response', true);
    this.$('#validation-panel').addClass('.hide').fadeOut(1000);
  },

  updateMission: function() {
      this.renderPartial(
        this.missionPanelTemplate, '#mission-panel', 'mission panel');
  },

  updateState: function() {
      this.renderPartial(
        this.statePanelTemplate, '#robot-state-panel', 'state panel');
  },

  updateTarget: function() {
      this.renderPartial(
        this.targetPanelTemplate, '#target-panel', 'target panel');
  },

  renderPartial: function(template, selector, debugMsg) {
    if (debugMsg !== undefined) console.log('Rendering ' + debugMsg);

    var context = template(this.model.toJSON());
    this.$(selector).html(context);
  },

  render: function() {

    this.$el.html(this.template(this.model.toJSON()));

    this.renderPartial(this.statePanelTemplate, '#robot-state-panel', 'state panel');
    this.renderPartial(this.targetPanelTemplate, '#target-panel', 'target panel');
    this.renderPartial(this.missionPanelTemplate, '#mission-panel', 'mission panel');

    if (this.waitingForValidation) {
      this.renderPartial(
        this.validationPanelTemplate, '#validation-panel', 'validation panel');
    }

    return this;
  },

  showVictimAlert: function() {
    console.log('Show victim alert');
    this.waitingForValidation = true;

    this.render();

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
