(() => {
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
  const cr = arr => {
    const {length} = arr
    if (0 === length) {
      return 0
    }
    const x_arr = []
    const y_arr = []
    for (const [x, y] of arr) {
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

  const
    xmlns = 'http://www.w3.org/2000/svg',
    msg_never_touch_me = 'Never touch me',
    never_touch_me = new Proxy({}, {
      get(_target, _p, _receiver) {
        throw msg_never_touch_me
      },
      set(_target, _p, _value, _receiver) {
        return false
      },
    }),
    add = Symbol('add'),
    cut = Symbol('cut'),
    new_svg = tag => {
      const
        view = document.createElementNS(xmlns, tag),
        svg_prefix = 'svg_',
        css_prefix = 'css_',
        view_key = 'view',
        accessors = {
          get: (_target, p, _receiver) => {
            if (view_key === p) {
              return view
            } else if (add === p) {
              return element => view.appendChild(element)
            } else if (cut === p) {
              return element => view.removeChild(element)
            } else {
              console.error(`${p} is touched unexpectedly.`)
              throw msg_never_touch_me
            }
          },
          set: (_target, p, value, _receiver) => {
            if (p.startsWith(svg_prefix)) {
              view.setAttributeNS(null, p.substr(4), value)
            } else if (p.startsWith(css_prefix)) {
              view.style[p.substr(4)] = value
            } else {
              view[p] = value
            }
            return true
          },
        }

      return new Proxy(never_touch_me, accessors)
    }

  class SC {
    #svgProxy = new_svg('svg')

    constructor() {
      this.#svgProxy.css_width = '300px'
      this.#svgProxy.css_height = '300px'
      this.#svgProxy.svg_viewBox = '0 0 100 100'
      document.body.appendChild(this.#svgProxy.view)
    }

    addChild(elementProxy) {
      this.#svgProxy[add](elementProxy.view)
      return this
    }

    cutChild(elementProxy) {
      this.#svgProxy[cut](elementProxy.view)
      return this
    }

    newPoint(x, y) {
      const circle = new_svg('circle')
      circle.svg_cx = x
      circle.svg_cy = y
      circle.svg_r = 1
      return circle
    }

    newLine(pairs) {
      const line = new_svg('polyline')
      line.svg_points = pairs.map(([x, y]) => `${x},${y}`).join(' ')
      line.svg_stroke = 'black'
      line.svg_fill = 'white'
      return line
    }
  }

  console.log(cr([[1, .2], [2, .3], [3, .6], [4, .8], [6, 30]]), '==== [[1, .2], [2, .3], [3, .6], [4, .8], [6, 30]]')
  console.log(cr([[1, .2], [1, .3], [1, .6], [1, .8], [1, 30]]), '==== [[1, .2], [1, .3], [1, .6], [1, .8], [1, 30]]')
  console.log(cr([[1, .1], [1, .1], [1, .1], [1, .1], [1, .1]]), '==== [[1, .1], [1, .1], [1, .1], [1, .1], [1, .1]]')
  console.log(cr([[1, 6], [2, 6], [3, 6], [4, 6], [5, 6]]), '==== [[1, 6], [2, 6], [3, 6], [4, 6], [5, 6]]')

  window.onload = () => {
    const
      svgCanvas = new SC(),
      firstPoint = svgCanvas.newPoint(20, 20)
    svgCanvas
      .addChild(svgCanvas.newPoint(50, 50))
      .addChild(firstPoint)
      .cutChild(firstPoint)
      .addChild(svgCanvas.newLine([[1, 10], [3, 4], [5, 6], [7, 8], [9, 10]]))
  }
})()
