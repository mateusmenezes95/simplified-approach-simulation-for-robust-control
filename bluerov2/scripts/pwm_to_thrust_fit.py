#!/usr/bin/env python3

from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from urllib.request import urlopen
from sklearn.metrics import r2_score, mean_squared_error
import sympy as sp


def polynomial_fit(x, y, order):
    from sympy.abc import u

    coefficients = np.polyfit(x, y, order)
    fitted_values = np.polyval(coefficients, x)

    r2_metric = r2_score(y, fitted_values)
    rmse = np.sqrt(mean_squared_error(y, fitted_values))
    print('R2 metric:', r2_metric)
    print('RMSE:', rmse)

    sp.init_printing()
    print('F(x) =', sp.Poly(np.poly1d(coefficients).coef, u).as_expr())

    plt.plot(x, y)
    plt.plot(x, fitted_values)
    plt.legend(['Data', 'Fit'])
    plt.xlabel('PWM Width [us]')
    plt.ylabel('Thrust [N]')
    plt.title('T200 Thrust vs PWM')
    plt.grid(True)
    plt.show()


if __name__ == '__main__':
    spreadsheet_filename = 't200-public-performance-data-10-20v-september-2019.xlsx'
    spreadsheet_filepath = Path(__file__).parent.parent / 'datalogs' / spreadsheet_filename
    print(spreadsheet_filepath)
    df = pd.read_excel(spreadsheet_filepath, sheet_name='16 V')

    order_of_polynomial = 7
    normalization_factor = 1000

    pwm_for_bat_with_sixteen_volts = df[' PWM (µs)'].to_list()
    pwm_normalized = [pwm / normalization_factor for pwm in pwm_for_bat_with_sixteen_volts]
    pwm_normalized_flipped = pwm_normalized[::-1]

    thrust_in_kgf = df[' Force (Kg f)'].to_list()
    thrust_in_newtons = [thrust * 9.807 for thrust in thrust_in_kgf]

    polynomial_fit(pwm_normalized, thrust_in_newtons, order_of_polynomial)
    polynomial_fit(pwm_normalized_flipped, thrust_in_newtons, order_of_polynomial)
