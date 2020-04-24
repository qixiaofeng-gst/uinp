const
repl = require('repl'),
server = repl.start({
  prompt: 'Damn: ',
  eval: (cmd, context, filename, callback) => {
    console.log(cmd, 'here we process customized commands')
    server.displayPrompt()
  },
})

server.defineCommand('test', {
  help: 'Test command, this type of command must prefix with a dot, eg. use .test to use this',
  action: () => {
    console.log('Test it')
  },
})

server.defineCommand('hello', {
  help: 'Hello command, welcome',
  action: () => {
    console.log('Hello ')
  },
})

server.defineCommand('bye', {
  help: 'Say bye',
  action: () => {
    console.log('Bye bye')
  },
})