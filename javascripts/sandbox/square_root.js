// Exponentiation: power = base^exponent, 幂=基数^指数
// Logarithm: exponent = log(base, power)
// The number to be find is base.

const exponent = 2
const power = parseFloat(process.argv[2] || 2)

const error = 1e-5
const half = .5

const violence_method = () => {
  const ts = Date.now()
  let low = 0
  let high = power
  const cal_base = () => ((low + high) * half)
  let base = cal_base()
  let i = 0
  while (++i) {
    const real_error = power - base * base
    if (Math.abs(real_error) < error) {
      break
    }
    if (real_error > 0) {
      low = base
    } else if (real_error === 0) {
      break
    } else {
      high = base
    }
    base = cal_base()
  }
  console.log(`[Violence Method] After ${Date.now() - ts}ms, looped ${i} times, calculated base: ${base}`)
}

const newton_method = () => {
  const ts = Date.now()
  // f(x) = x^2 - a
  // f'(x) = 2x
  // x_1 = x_0 - f(x_0)/f'(x_0)
  // There is a more simple method: fast inverse square root
  const f_x = x => (x * x - power)
  const f_x_1 = x => (2 * x)
  const err = x => (f_x(x) / f_x_1(x))
  let base = power >> (Math.floor(Math.floor(power).toString(2).length * half))
  let i = 0
  while (++i) {
    const current_error = err(base)
    base = base - current_error
    if (Math.abs(current_error) < error) {
      break
    }
  }
  console.log(`[Newton Method] After ${Date.now() - ts}ms, looped ${i} times, calculated base: ${base}`)
}

const manual_method = () => {
  const ten = 10
  const twenty = ten * 2
  const one_handred = ten * ten
  const get_max_int_root = int_x => {
    if (int_x > 15) {
      for (let root = 9; root > 3; --root) {
        const square = root * root
        if (square <= int_x) {
          return { root, rest: int_x - square }
        }
      }
    } else {
      for (let root = 3; root >= 0; --root) {
        const square = root * root
        if (square <= int_x) {
          return { root, rest: int_x - square }
        }
      }
    }
  }
  const init = init_num => {
    const { root, rest } = get_max_int_root(init_num)
    root_int = root
    current_sum = root
    current_rest = rest
  }
  const get_position = (sum, prev_rest) => {
    for (let i = 9; i >= 0; --i) {
      const rest = prev_rest - (20 * sum + i) * i
      if (rest >= 0) {
        return { rest, root: i }
      }
    }
    return { rest: 0, root: 0 }
  }
  const forward = appendix => {
    current_rest = current_rest * one_handred + appendix
    const { root, rest } = get_position(current_sum, current_rest)
    current_sum = current_sum * 10 + root
    current_rest = rest
    return root
  }
  const forward_int = appendix => {
    root_int = root_int * ten + forward(appendix)
  }
  const forward_dec = appendix => {
    root_dec = root_dec * ten + forward(appendix)
  }
  const calc = (cb_forward, start_index, str_form_num) => {
    const length = str_form_num.length
    for (let i = start_index; i < length; i += 2) {
      const appendix = parseInt(str_form_num.substr(i, 2))
      cb_forward(appendix)
      if (current_rest === 0) {
        break
      }
    }
  }
  const calc_decimal = () => calc(forward_dec, 0, decimal)
  const calc_integer = start_index => calc(forward_int, start_index, int_part)
  const decimal_length = 10
  const str_form = power.toFixed(decimal_length)
  const [int_part, decimal] = str_form.split('.')
  const ts = Date.now()
  let root_int = 0
  let root_dec = 0
  let current_sum = 0
  let current_rest = 0
  if (int_part.length < 3) {
    init(parseInt(int_part))
    if (current_rest > 0) {
      calc_decimal()
    }
  } else {
    const is_even_length = 0 === (int_part.length % 2)
    if (is_even_length) {
      calc_integer(0)
    } else {
      init(parseInt(int_part[0]))
      calc_integer(1)
    }
    calc_decimal()
  }
  console.log(`[Manual Method] after ${Date.now() - ts}ms, a constant loop times related with expected precision, calculated base: ${root_int}.${root_dec}`)
}

violence_method()
newton_method()
manual_method()

//TODO study statistics characteristic of above methods
