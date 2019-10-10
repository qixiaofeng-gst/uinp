# NPM part

## Override install

```
npm i any-module
npm i any-module -D
```

The second command above will override the configuration in `package.json` made by first command, which means that the entry generated in `dependencies` part will be removed and an new entry will be added to `devDependencies`.

# Webpack part

## Run without configuration

A simple `npx webpack` command will do it. Just make sure the entry `src/index.js` exists. A `dist` directory will be auto-generated and there will be a `main.js` in it. If `src/index.js` is missing, the command will be failed.
