'use strict';

var Backbone = require('backbone');
var Dispatcher = require('../dispatcher');

/**
 * View template.
 */

var agentTemplate = require('../templates/agent.hbs');


var AgentView = Backbone.View.extend({

  el: $('#agent-info'),

  template: agentTemplate,

  initialize: function() {
    console.log('View Agent initialized');

    this.model.subscribe();
    Dispatcher.on(this.model.get('dispatcherTopic'), this.render, this);
  },

  render: function() {

    this.$el.html(this.template(this.model.toJSON()));

    return this;
  }

});


module.exports = AgentView;
