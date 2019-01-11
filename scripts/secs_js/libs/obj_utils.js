
const set_fn_as_key = obj => {
  for (const k in obj) {
    obj[k].key = k
  }
}

const calc_margin = obj => {
  let margin = 0
  for (const k in obj) {
    margin = Math.max(margin, k.length)
  }
  return margin
}

const calc_margin_arr = (obj_arr, key) => {
  let margin = 0
  for (const o of obj_arr) {
    margin = Math.max(margin, o[key].length)
  }
  return margin
}

module.exports = {
  set_fn_as_key,
  calc_margin,
  calc_margin_arr
}
