const {
  the,
} = require('test_js')

const
test_throw = () => {
  the(true).eq('true')
},
test_true = () => {
  the(true).eq(true)
},
test_throw_again = () => {
  the(true).eq('true')
}

module.exports = {
  test_throw,
  test_true,
  test_throw_again,
}
