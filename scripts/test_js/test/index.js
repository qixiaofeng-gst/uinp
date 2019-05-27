const {
  the,
  run,
} = require('../index')

run({
  test_throw: () => {
    the(true).eq('true')
  }
}, 'Test4Test')
