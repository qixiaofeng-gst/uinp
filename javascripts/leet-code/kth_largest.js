/**
 * @param {number} k
 * @param {number[]} nums
 */
const KthLargest = function(k, nums) {
  this.k = k
  this.nums = nums
  nums.sort((a, b) => b - a)
}

/** 
 * @param {number} val
 * @return {number}
 */
KthLargest.prototype.add = function(val) {
  let inserted = false
  for (let i = 0; i < this.nums.length; ++i) {
    if (val > this.nums[i]) {
      inserted = true
      this.nums.splice(i, 0, val)
      break
    }
  }
  if (false == inserted) {
      this.nums.push(val)
  }
  return this.nums[this.k - 1]
}

const obj = new KthLargest(3, [5, 4, 6, 7, 8])
console.log(obj.add(1))
console.log(obj.add(10))
console.log(obj.add(11))