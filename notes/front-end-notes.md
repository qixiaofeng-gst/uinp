# NPM part

## Override install

```
npm i any-module
npm i any-module -D
```

The second command above will override the configuration in `package.json` made by first command, which means that the entry generated in `dependencies` part will be removed and an new entry will be added to `devDependencies`.

---

# Webpack part

## Run without configuration

A simple `npx webpack` (**C1**) command will do it. Just make sure the entry `src/index.js` exists. A `dist` directory will be auto-generated and there will be a `main.js` in it. If `src/index.js` is missing, the command will be failed.

## Run with configuration

It is `npx webpack --config path/to/config.js` (**C2**) command. Put it under the `scripts` part of `package.json` could save the job.

## Notes on the configuration

### entry

Wherever the location of the configuration script, the value of the entry is only relative to the `pwd` running the **C2** command.

### output path

The value of output path has to be an absolute path, otherwise there will be an error thrown while use **C1/2** command.
