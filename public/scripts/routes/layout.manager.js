'use strict';

var _ = require('underscore');


function _classCallCheck(instance, Constructor) {
  if (!(instance instanceof Constructor)) {
    throw new TypeError('Cannot call a class as a function');
  }
}

/**
 * Class to register layouts
 */

var LayoutManager = (function () {

  function LayoutManager(options) {
    _classCallCheck(this, LayoutManager);

    this.layouts = options.layouts;
    this.renderedLayouts = [];
  }

  LayoutManager.prototype.isRendered = function isRendered(layout) {
    return this.renderedLayouts.indexOf(layout) > -1;
  },

  LayoutManager.prototype.render = function render(layout) {
    var _this = this;
    _.each(this.layouts, function(layoutObject, layoutName, list) {
      if (layoutName === layout) {
        _this.renderedLayouts.push(layoutName);
        layoutObject.render();
      } else {
        //layoutObject.remove();
      }
    });
  };

  LayoutManager.prototype.remove = function remove(layout) {
  };

  return LayoutManager;
})();

module.exports = LayoutManager;
