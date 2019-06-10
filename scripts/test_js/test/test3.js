const {
  the,
} = require('../index')

const
test_true = () => {
  the(true).eq(true)
}

module.exports = {
  test_true,
}
