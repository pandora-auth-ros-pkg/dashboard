
install:
	npm install

global:
	npm install -g browserify
	npm install -g watchify
	npm install -g jasmine
	npm install -g gulp

watch-js:
	@watchify -t browserify-handlebars ./public/scripts/app.js -o ./public/build/bundle.js -v

test:
	@jasmine

clean:
	rm -rf ./public/build

.PHONY: install
