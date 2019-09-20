const num_count = 1000
const to_sort = []

const generate_to_sort = () => {
  const arr = []
  for (let i = 0; i < num_count; ++i) {
    arr.push(i)
  }
  for (let i = 0; i < num_count; ++i) {
    const index = Math.floor(Math.random() * arr.length)
    to_sort.push(arr.splice(index, 1)[0])
  }
}
const quicksort = () => {
  const swap = (a, b) => {
    const tmp = to_sort[a]
    to_sort[a] = to_sort[b]
    to_sort[b] = tmp
  }
  const inner_sort = (start, end) => {
    if (start >= end) {
      return
    }
    
    let pivot = start
    const pivot_value = to_sort[pivot]
    for (let i = start + 1; i <= end; ++i) {
      const current_value = to_sort[i]
      if (pivot_value < current_value) {
        continue
      }
      const neighbor = pivot + 1
      if (i > neighbor) {
        swap(i, neighbor)
      }
      swap(neighbor, pivot)
      pivot = neighbor
    }
    inner_sort(start, pivot - 1)
    inner_sort(pivot + 1, end)
  }
  inner_sort(0, to_sort.length - 1)
}
const print_to_sort = () => {
  for (const i of to_sort) {
    console.log(i)
  }
}

generate_to_sort()
quicksort()
print_to_sort()