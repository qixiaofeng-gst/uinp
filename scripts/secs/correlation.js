(() => {
  const
    empty = '',
    space = ' ',
    precision = 2,
    color_base = 0xeeeeee,
    clamp_str = (str, width = 6) => str.length > width ? str.substr(0, width) : str + `${(() => {
      const limit = width - str.length
      let result = empty
      for (let i = 0; i < limit; ++i) {
        result += '0'
      }
      return result
    })()}`,
    random_color = () => `#${clamp_str(Math.ceil(Math.random() * color_base).toString(16))}`,
    pairs2lists = pairs => {
      const
        listA = [],
        listB = [],
        lists = [listA, listB]
      pairs.forEach(([a, b]) => {
        listA.push(a)
        listB.push(b)
      })
      return lists
    },
    lists2pairs = lists => {
      const
        [listA, listB] = lists,
        pairs = []
      const limit = Math.min(listA.length, listB.length)
      for (let i = 0; i < limit; ++i) {
        pairs.push([listA[i], listB[i]])
      }
      return pairs
    },
    calc_average = arr => (arr.reduce((sum, curr) => sum + curr, 0) / arr.length),
    calc_standard_deviation = arr => {
      const arr_avg = calc_average(arr)
      const sum_d = (sum, curr) => sum + Math.pow(curr - arr_avg, 2)
      return Math.sqrt(arr.reduce(sum_d, 0) / arr.length)
    },
    // en: use it as a action at every element
    calc_standard_unit = arr => {
      const arr_sd = calc_standard_deviation(arr)
      const arr_avg = calc_average(arr)
      return (
        0 === arr_sd ?
          arr.map(() => 1) : // here 0 and 1 mean same
          arr.map(num => (num - arr_avg) / arr_sd)
      )
    },
    calc_correlation_for_lists = lists => {
      const [x_arr, y_arr] = lists
      const limit = Math.min(x_arr.length, y_arr.length)
      const x_arr_in_su = calc_standard_unit(x_arr)
      const y_arr_in_su = calc_standard_unit(y_arr)
      let sum = 0
      for (let i = 0; i < limit; ++i) {
        sum += (x_arr_in_su[i] * y_arr_in_su[i])
      }
      return (sum / limit).toPrecision(precision)
    },
    calc_correlation_for_pairs = pairs => {
      return calc_correlation_for_lists(pairs2lists(pairs))
    },
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
    view = Symbol('view'),
    new_svg = tag => {
      const
        element = document.createElementNS(xmlns, tag),
        svg_prefix = 'svg_',
        css_prefix = 'css_',
        _2dash = str => str.replace(/_/g, '-'),
        accessors = {
          get: (_target, p, _receiver) => {
            if (view === p) {
              return element
            } else if (add === p) {
              return child => element.appendChild(child)
            } else if (cut === p) {
              return child => element.removeChild(child)
            } else {
              console.error(`${p} is touched unexpectedly.`)
              throw msg_never_touch_me
            }
          },
          set: (_target, p, value, _receiver) => {
            if (p.startsWith(svg_prefix)) {
              element.setAttributeNS(null, _2dash(p.substr(4)), value)
            } else if (p.startsWith(css_prefix)) {
              element.style[p.substr(4)] = value
            } else {
              element[p] = value
            }
            return true
          },
        }

      return new Proxy(never_touch_me, accessors)
    }

  class SC {
    #svgProxy = new_svg('svg')
    #width = 800
    #height = 300

    constructor() {
      this.#svgProxy.css_width = `${this.#width}px`
      this.#svgProxy.css_height = `${this.#height}px`
      this.#svgProxy.svg_viewBox = `0 0 ${this.#width} ${this.#height}`
      document.body.appendChild(this.#svgProxy[view])
    }

    addChild(elementProxy) {
      this.#svgProxy[add](elementProxy[view])
      return this
    }

    cutChild(elementProxy) {
      this.#svgProxy[cut](elementProxy[view])
      return this
    }

    newPoint(x, y) {
      const circle = new_svg('circle')
      circle.svg_cx = x
      circle.svg_cy = y
      circle.svg_r = 1
      return circle
    }

    newLine(lineValues) {
      const
        limit = lineValues.length,
        line = new_svg('polyline'),
        increment = this.#width / (limit - 1),
        coords = []
      for (let i = 0, x = 0; i < limit; ++i, x += increment) {
        coords.push(`${x},${lineValues[i]}`)
      }

      line.svg_points = coords.join(space)
      line.svg_stroke = random_color()
      line.svg_stroke_width = 1
      line.svg_fill_opacity = 0
      return line
    }
  }

  console.log(calc_correlation_for_pairs([[1, .2], [2, .3], [3, .6], [4, .8], [6, 30]]), 'expected: 0.8239')
  console.log(calc_correlation_for_pairs([[1, .2], [1, .3], [1, .6], [1, .8], [1, 30]]), 'expected: 0.0000')
  console.log(calc_correlation_for_pairs([[1, .1], [1, .1], [1, .1], [1, .1], [1, .1]]), 'expected: 1.0000')
  console.log(calc_correlation_for_pairs([[1, 6], [2, 6], [3, 6], [4, 6], [5, 6]]), 'expected: 0.0000')

  window.onload = () => {
    const
      svgCanvas = new SC(),
      outputText = document.createElement('span'),
      firstPoint = svgCanvas.newPoint(20, 20),
      generateRandomValues = () => {
        const
          limit = 260,
          valueDomain = 300,
          result = []

        for (let x = 0; x < limit; ++x) {
          result.push(Math.ceil(Math.random() * valueDomain))
        }
        return result
      },
      lineAValues = generateRandomValues(),
      lineBValues = generateRandomValues(),
      toCalc = [lineAValues, lineBValues]

    document.body.appendChild(outputText)
    outputText.innerHTML = `cr of pairs to draw: ${calc_correlation_for_lists(toCalc)}`
    svgCanvas
      .addChild(svgCanvas.newPoint(50, 50))
      .addChild(firstPoint)
      .cutChild(firstPoint)
      .addChild(svgCanvas.newLine(lineAValues))
      .addChild(svgCanvas.newLine(lineBValues))
  }
})()
