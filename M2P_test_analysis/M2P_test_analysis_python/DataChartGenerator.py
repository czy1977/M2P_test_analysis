from plotly import __version__
from plotly.offline import download_plotlyjs, init_notebook_mode, plot, iplot
import os
from plotly import tools, io
import plotly.graph_objs as go
import numpy as np
import pandas as pd
import math
from scipy.interpolate import interp1d
from plotly.subplots import make_subplots
import argparse
from numpy.linalg import norm

file_extention = ".csv"

arg = argparse.ArgumentParser("This is data chart generator for superimposition test analysis! give directory with csv log.")

def get_filenames(path):
    file_list = []
    for root, dirs, files in os.walk(path):
        for name in files:
            if name.endswith(file_extention):
                file_list.append(os.path.join(root,name).replace(path,''))

    return file_list


def main(arg):
    prefix = './logs/0319/'  # set the folder used with CSV logs

    prefix = args.directory
    prefix +='/'

    # the file number in the
    #file_index_begin = 1
    #file_index_end = 20

    # filenameTemplate = 'log_c{0:0>4d}.csv'
    #filename_template = 'log_{0:0>2d}.csv'
    #filenames = [filename_template.format(s) for s in range(file_index_begin, file_index_end + 1)]

    try:
        listfiles = get_filenames(prefix)

    except IOError:
        return 0
    else:
        filenames = (listfiles)




    min_time_offset = -10
    max_time_offset = 25
    max_iteration = max_time_offset - min_time_offset

    for filename in filenames:
        roxindex = 1
        fig = make_subplots(rows=7, row_heights=[1000 for c in range(0, 7)], subplot_titles=[
            'original position log',
            'position error in pixel',
            'position error in mm',
            'Position error on latency simulation',
            'Position after latency match',
            'X position error in pixel after latency match',
            'Y position error in pixel after latency match'],
                            vertical_spacing=0.02
                            )
        try:
            filepath = os.path.join(prefix , filename)
            data_log = pd.read_csv(filepath, header=0, index_col=False)
        except IOError:
            print(filepath + " load failed")
            continue;
        except FileNotFoundError:
            print(filepath + " file not found")
            continue;
        else:
            print(filepath + " load succeed")

        # show the position data diagram
        p = []
        # index = np.arange(0,len(data_log))
        index = data_log['frame_id']

        for c in ['real_x', 'real_y', 'expected_x', 'expected_y']:
            trace = go.Scatter(
                y=data_log[c],
                x=index,
                name=c
            )
            p.append(trace)

        axis_template = dict(
            showgrid=True,  # 网格
            zeroline=True,  # 是否显示基线,即沿着(0,0)画出x轴和y轴
            nticks=20,
            showline=True,
            # title='X axis',
            # mirror='all',
            zerolinecolor="#FF0000"
        )
        layout = go.Layout(
            xaxis=axis_template,
            yaxis=axis_template,
            title='original position log'
        )
        fig.update_layout(dict(height=5000, xaxis=axis_template, yaxis=axis_template))
        fig.add_traces(p, rows=[roxindex, roxindex, roxindex, roxindex], cols=[1, 1, 1, 1])
        roxindex += 1
        '''
        fig=go.Figure(
            data=p,
            layout=layout
        )
        '''


        # show the visual & real position error  in pixel
        data_diff_x = np.subtract(data_log['real_x'], data_log['expected_x'])
        data_diff_y = np.subtract(data_log['real_y'], data_log['expected_y'])

        p = []
        # index = np.arange(0,len(data_log))
        index = data_log['frame_id']

        p.append(go.Scatter(
            y=data_diff_x,
            x=index,
            name='diff x in pixel'

        ))

        p.append(go.Scatter(
            y=data_diff_y,
            x=index,
            name='diff y in pixel'
        ))

        layout.title = 'position error in pixel'

        '''    
        fig=go.Figure(
            data=p,
            layout=layout
        )
        '''
        fig.add_traces(p, rows=[roxindex, roxindex], cols=[1, 1])
        roxindex += 1

        # get the uv error digram
        mark_width = 250  # 250mm
        mark_height = 150  # 150mm
        expected_u = np.mean(data_log['u'][0:10])
        expected_v = np.mean(data_log['v'][0:10])
        real_u = data_log['u']
        real_v = data_log['v']

        data_diff_u_dist = np.subtract(real_u, expected_u) * mark_width
        data_diff_v_dist = np.subtract(real_v, expected_v) * mark_height

        p = []
        # index = np.arange(0,len(data_log))
        index = data_log['frame_id']

        p.append(go.Scatter(
            y=data_diff_u_dist,
            x=index,
            name='diff x in mm'
        ))
        p.append(go.Scatter(
            y=data_diff_v_dist,
            x=index,
            name='diff y in mm'
        ))
        layout.title = 'position error in mm'
        fig.add_traces(p, rows=[roxindex, roxindex], cols=[1, 1])
        roxindex += 1

        # calculate the latency by shift the element and calculate the var
        expected_x = data_log['expected_x']
        expected_y = data_log['expected_y']
        real_x = data_log['real_x']
        real_y = data_log['real_y']
        index = data_log['frame_id']

        diff_x_sums = []
        diff_y_sums = []

        len_of_list = len(real_x) - max_iteration

        expected_x = expected_x[0:len_of_list]
        expected_y = expected_y[0:len_of_list]

        p = []
        for it in range(min_time_offset, max_time_offset + 1):
            shifted_realx = real_x.shift(-it)
            shifted_realy = real_y.shift(-it)
            shifted_realx = shifted_realx[0:len_of_list]
            shifted_realy = shifted_realy[0:len_of_list]

            diffx = np.subtract(shifted_realx, expected_x)
            diffy = np.subtract(shifted_realy, expected_y)

            diff_sum_x = np.nanmean(np.abs(diffx))
            diff_sum_y = np.nanmean(np.abs(diffy))

            diff_x_sums.append(diff_sum_x)
            diff_y_sums.append(diff_sum_y)

        diff_index = np.arange(min_time_offset, max_time_offset + 1)

        min_error_x = np.argmin(diff_x_sums) + min_time_offset
        min_error_y = np.argmin(diff_y_sums) + min_time_offset

        text_x = [i if (i == min_error_x) else '' for i in diff_index]
        text_y = [i if (i == min_error_y) else '' for i in diff_index]
        p.append(go.Scatter(
            y=diff_x_sums,
            x=diff_index,
            mode="lines+text",
            name='total error X',
            text=text_x,
            textposition="bottom center"
        ))
        p.append(go.Scatter(
            y=diff_y_sums,
            x=diff_index,
            mode="lines+text",
            name='total error Y',
            text=text_y,
            textposition="bottom center"
        ))
        layout.xaxis['title'] = 'latency'
        layout.yaxis['title'] = 'total position error'
        layout.title = 'Position error on latency simulation'
        '''
        fig=go.Figure(
            data=p,
            layout=layout
        )
        '''
        fig.add_traces(p, rows=[roxindex, roxindex], cols=[1, 1])
        roxindex += 1


        # draw the diagramm after best match
        best_match_latency_x = np.argmin(diff_x_sums) + min_time_offset
        best_match_latency_y = np.argmin(diff_y_sums) + min_time_offset
        # best_match_latency = min(best_match_latency_x,best_match_latency_y)
        range_of_diffx = abs(max(data_log['real_x']) - min(data_log['real_x']))
        range_of_diffy = abs(max(data_log['real_y']) - min(data_log['real_y']))
        if (range_of_diffx > range_of_diffy):
            best_match_latency = best_match_latency_x
        else:
            best_match_latency = best_match_latency_y

        shifted_realx = real_x.shift(-best_match_latency)
        shifted_realx = shifted_realx[0:len_of_list]

        shifted_realy = real_y.shift(-best_match_latency)
        shifted_realy = shifted_realy[0:len_of_list]

        shifted_index = data_log['frame_id'].shift(-best_match_latency)
        shifted_index = shifted_index[0:len_of_list]

        p = []
        trace = go.Scatter(
            y=shifted_realx,
            x=shifted_index,
            name='shifted real_x'

        )
        p.append(trace)

        trace = go.Scatter(
            y=expected_x,
            x=shifted_index,
            name='expected_x'
        )
        p.append(trace)

        trace = go.Scatter(
            y=shifted_realy,
            x=shifted_index,
            name='shifted real_y'

        )
        p.append(trace)

        trace = go.Scatter(
            y=expected_y,
            x=shifted_index,
            name='expected_y'
        )
        p.append(trace)

        layout.xaxis['title'] = 'time'
        layout.yaxis['title'] = 'position'
        layout.title = 'Position after latency match'
        '''fig=go.Figure(
            data=p,
            layout=layout
        )'''
        fig.add_traces(p, rows=[roxindex, roxindex, roxindex, roxindex], cols=[1, 1, 1, 1])
        roxindex += 1

        # show the visual & real position error  in pixel
        data_diff_x_shifted = np.subtract(shifted_realx, expected_x)
        data_diff_x = np.subtract(data_log['real_x'], data_log['expected_x'])
        data_diff_index = shifted_index

        p = []
        index = index = data_log['frame_id']
        p.append(go.Scatter(
            y=data_diff_x,
            x=index,
            name='original error x'

        ))
        p.append(go.Scatter(
            y=data_diff_x_shifted,
            x=data_diff_index,
            name='after match errorx'

        ))

        layout.title = 'X position error in pixel after latency match'
        fig.add_traces(p, rows=[roxindex, roxindex], cols=[1, 1])
        roxindex += 1
        '''
        fig=go.Figure(
            data=p,
            layout=layout
        )
        '''


        data_diff_y_shifted = np.subtract(shifted_realy, expected_y)
        data_diff_y = np.subtract(data_log['real_y'], data_log['expected_y'])
        data_diff_index = shifted_index

        p = []
        index = data_log['frame_id']
        p.append(go.Scatter(
            y=data_diff_y,
            x=index,
            name='original error y'

        ))
        p.append(go.Scatter(
            y=data_diff_y_shifted,
            x=data_diff_index,
            name='after match error y'

        ))

        layout.title = 'Y position error in pixel after latency match'
        '''
        fig=go.Figure(
            data=p,
            layout=layout
        )
        '''
        fig.add_traces(p, rows=[roxindex, roxindex], cols=[1, 1])
        roxindex += 1

        # io.write_html(fig, file=prefix+filename.replace('.csv','_')+'datachart'+'.html')
        output_filepath = os.path.join(prefix , filename.replace('.csv', '_') + 'datachart' + '.html')
        io.write_html(fig, include_plotlyjs='cdn', file=output_filepath)
        print("write %s finish" %(output_filepath))


if __name__ == '__main__':
    arg.add_argument("--directory", "-d", default="./logs/", type=str, required=True, help="the folder of your log input files.")
    args = arg.parse_args()
    main(args)
