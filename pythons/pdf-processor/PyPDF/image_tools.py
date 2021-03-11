import zlib
import cv2 as _cv
import numpy as _np


def chop_off_image_empty_edges(meta, data, image_index, should_output_chopped_images=False):
    width = meta[b'/Width']
    height = meta[b'/Height']
    raw: _np.ndarray = _np.frombuffer(zlib.decompress(data), dtype=_np.uint8)
    raw = raw.reshape((height, width, -1))
    component_count = len(raw[0][0])
    top_margin, bottom_margin, left_margin, right_margin = scan_margins(raw, 254)
    raw = raw[top_margin:(height - bottom_margin), left_margin:(width - right_margin), :]
    new_width = len(raw[0])
    new_height = len(raw)
    new_original_length = new_width * new_height * component_count
    compressed = zlib.compress(raw.reshape(new_original_length).tobytes())
    new_compressed_length = len(compressed)
    # _u.debug(
    #     'length of data:', len(data),
    #     '| expected length:', width * height * component_count,
    #     '| chopped length:', len(),
    #     '| self:', self,
    # )
    # _u.stacktrace_debug()
    if should_output_chopped_images:
        _cv.imwrite('output/page_{}.jpg'.format(image_index), raw)
    return new_width, new_height, new_compressed_length, compressed


def _mean(array_1d: _np.ndarray):
    return array_1d.mean()


def _scan_from_edge(raw: _np.ndarray, threshold, iterator, callback_get_component_line):
    line_count = 0
    for i in iterator:
        if (
                _mean(callback_get_component_line(raw, i, 0)) < threshold or
                _mean(callback_get_component_line(raw, i, 1)) < threshold or
                _mean(callback_get_component_line(raw, i, 2)) < threshold
        ):
            if line_count > 1:
                return line_count - 1
            else:
                return 0
        line_count += 1
    return 0


def _get_horizontal_component_line(nparray, line_index, component_index):
    return nparray[line_index, :, component_index]


def _get_vertical_component_line(nparray, line_index, component_index):
    return nparray[:, line_index, component_index]


def scan_margins(raw: _np.ndarray, threshold=250):
    return (
        _scan_from_edge(raw, threshold, range(len(raw)), _get_horizontal_component_line),  # Top
        _scan_from_edge(raw, threshold, reversed(range(len(raw))), _get_horizontal_component_line),  # Bottom
        _scan_from_edge(raw, threshold, range(len(raw[0])), _get_vertical_component_line),  # Left
        _scan_from_edge(raw, threshold, reversed(range(len(raw[0]))), _get_vertical_component_line),  # Right
    )
