/* 
 * Copyright (C) 2025 Simon D. Levy
 *
 *  This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 *  This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#include <cstdlib>

#include <levy_snn_util.hpp>

static const char * NETWORK = "/home/levys/Desktop/framework/networks/difference_risp_train.txt";

static const size_t NITER = 100;
static const double divisor = 50;
static const double offset = 0;

static double randval()
{

    return 2 * (double)rand() / INT_MAX - 1;
}

int main(int argc, char ** argv)
{
    (void)argc;
    (void)argv;

    static SNN * snn = new SNN(NETWORK, "risp");

    for (size_t k=0; k<100; ++k) {

        vector<double> observations = { randval(), randval() };

        vector <double> actions = {};

        snn->step(observations, actions);

        printf("%+3.3f - %+3.3f = %+3.3f\n", 
                observations[0], observations[1], actions[0]);
    }

}
