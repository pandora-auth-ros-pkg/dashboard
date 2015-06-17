'use strict';

var Backbone = require('backbone');
var Dispatcher = require('../dispatcher');

/**
 * View template.
 */

var agentTemplate = require('../templates/agent.hbs');
var victimNotification = require('../templates/victim.alert.hbs');
var victimDialog = require('../templates/victim.dialog.hbs');


var AgentView = Backbone.View.extend({

  template: agentTemplate,
  victimNotificationTemplate: victimNotification,
  victimDialogTemplate: victimDialog,

  initialize: function() {
    console.log('View Agent initialized');

    this.model.subscribe();
    Dispatcher.on(this.model.get('dispatcherTopic'), this.render, this);
    Dispatcher.on('agent:victim:alert', this.showVictimAlert, this);
  },

  render: function() {

    this.$el.html(this.template(this.model.toJSON()));

    return this;
  },

  showVictimAlert: function() {
    console.log('Show victim alert');

    // Show the modal.
    var context = this.victimDialogTemplate(this.model.toJSON());
    $('#victim-alert').html(context);
    $('#victim-alert').modal('show');

    // Show alert on the navbar
    context = this.victimNotificationTemplate(this.model.toJSON());
    $('.alert-feed ul.dropdown-menu li.no-alerts').remove();
    $('.alert-feed ul.dropdown-menu').append(context);

    // Increase by one the number of notifications.
    $('.alert-feed .badge').text(Number($('.alert-feed .badge').text()) + 1);

    return this;
  }

});


module.exports = AgentView;
