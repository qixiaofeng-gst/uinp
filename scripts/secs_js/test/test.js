const {
  the,
} = require('test_js')

const
a = 0,
b = 1,
c = 2,
test_throw = () => {
  the(true).eq('true')
}

module.exports = {
  a, b, c,
  test_throw,
}
