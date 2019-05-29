const
fs = require('fs'),
{
  run,
} = require('../index')

const
ls = fs.readdirSync(__dirname),
filter_index = 'index.js'

for (const f of ls) {
  if (f == filter_index) {
    continue
  }
  run(require(`${__dirname}/${f}`), f)
}
