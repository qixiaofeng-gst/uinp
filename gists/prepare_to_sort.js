module.exports = {
  prepare: num_count => {
    const to_sort = []

    const arr = []
    for (let i = 0; i < num_count; ++i) {
      arr.push(i)
    }
    for (let i = 0; i < num_count; ++i) {
      const index = Math.floor(Math.random() * arr.length)
      to_sort.push(arr.splice(index, 1)[0])
    }
    
    return to_sort
  },
  print: to_sort => {
    for (const i of to_sort) {
      console.log(i)
    }
  },
  swap: (to_sort, a, b) => {
    const tmp = to_sort[a]
    to_sort[a] = to_sort[b]
    to_sort[b] = tmp
  },
}
