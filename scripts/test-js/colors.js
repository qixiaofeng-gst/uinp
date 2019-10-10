const
colors = {
  reset: '\x1b[0m',
  bright: '\x1b[1m',
  dim: '\x1b[2m',
  underscore: '\x1b[4m',
  blink: '\x1b[5m',
  reverse: '\x1b[7m',
  hidden: '\x1b[8m',

  fg_black: '\x1b[30m',
  fg_red: '\x1b[31m',
  fg_green: '\x1b[32m',
  fg_yellow: '\x1b[33m',
  fg_blue: '\x1b[34m',
  fg_magenta: '\x1b[35m',
  fg_cyan: '\x1b[36m',
  fg_white: '\x1b[37m',

  bg_black: '\x1b[40m',
  bg_red: '\x1b[41m',
  bg_green: '\x1b[42m',
  bg_yellow: '\x1b[43m',
  bg_blue: '\x1b[44m',
  bg_magenta: '\x1b[45m',
  bg_cyan: '\x1b[46m',
  bg_white: '\x1b[47m',
},
make = str => {
  str = str || ''
  
  const
  key_out = 'str',
  prefix_fg = 'fg_',
  reset = colors.reset
  
  return new Proxy({}, {
    get: (_, key, __) => {
      if (key_out == key) {
        return () => str
      } else if (colors.hasOwnProperty(key)) {
        return next => make(str + colors[key] + next + reset)
      } else {
        const key_with_fg = prefix_fg + key
        if (colors.hasOwnProperty(key_with_fg)) {
          return next => make(str + colors[key_with_fg] + next + reset)
        } else {
          throw new Error(`Colors object does not have property [${key}].`)
        }
      }
    },
    set: (_, key, val, __) => {
      throw new Error('Colors object does not support any assignment statement.')
    },
  })
}

/**
cstr.red('haha').cyan('hehe').bright('hoho').str()
TODO cstr.make(str, { red: true, blink: true }).str()
*/

/**XXX Seems that could not export a proxy XXX*/
module.exports = make
