const avg = arr => (arr.reduce((sum, curr) => sum + curr, 0) / arr.length)
const sd = arr => {
  const arr_avg = avg(arr)
  const sum_d = (sum, curr) => sum + Math.pow(curr - arr_avg, 2)
  return Math.sqrt(arr.reduce(sum_d, 0) / arr.length)
}
// en: use it as a action at every element
const en_sd = arr => {
  const arr_sd = sd(arr)
  const arr_avg = avg(arr)
  return (
    0 === arr_sd ?
    arr.map(() => 1) : // here 0 and 1 mean same
    arr.map(num => (num - arr_avg) / arr_sd)
  )
}
const r = arr2 => {
  const { length } = arr2
  if (0 === length) {
    return 0
  }
  const x_arr = []
  const y_arr = []
  for (const [x, y] of arr2) {
    x_arr.push(x)
    y_arr.push(y)
  }
  // su: Standard Unit
  const x_arr_in_su = en_sd(x_arr)
  const y_arr_in_su = en_sd(y_arr)
  let sum = 0
  for (let i = 0; i < length; ++i) {
    sum += (x_arr_in_su[i] * y_arr_in_su[i])
  }
  return (sum / length).toFixed(4)
}

//console.log(r([[1, .2], [2, .3], [3, .6], [4, .8], [6, 30]]), '==== 0.82')
//console.log(r([[1, .2], [1, .3], [1, .6], [1, .8], [1, 30]]), '====')
//console.log(r([[1, .1], [1, .1], [1, .1], [1, .1], [1, .1]]), '====')
console.log(r([[1, 6], [2, 6], [3, 6], [4, 6], [5, 6]]), '====')
