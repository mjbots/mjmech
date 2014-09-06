# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

import numpy

def is_positive_definite(P):
    return numpy.all(numpy.linalg.eigvals(P) > 0)

def make_positive_definite(P):
    '''Return a matrix which is "near" to P, but is positive definite.
    i.e. all of its eigenvalues will be positive.'''

    if not is_positive_definite(P):
        print numpy.linalg.eigvals(P)
        raise RuntimeError('not positive definite!')
    # TODO jpieper: Implement me when we first find a problem.

    return P

class UkfFilter(object):
    '''An unscented Kalman filter, as described in Optimal State
    Estimation, by Dan Simon.  2006'''
    def __init__(self, initial_state, initial_covariance,
                 process_function=None, process_noise=None,
                 measurement_function=None, measurement_noise=None,
                 covariance_limit=None):
        '''Construct a new UkfFilter.

         @param initial_state - numpy column vector describing initial state
         @param initial_covariance - numpy square covariance matrix of
                the same dimension as initial_state
         @param process_function - if specified, a default process function,
                which must take two arguments, (state, dt_s)
         @param process_noise - if specified, a square matrix to be added
                to the covariance after each state update (after scaling by
                time)
         @param covariance_limit - a method which takes and returns a
                covariance, it may be used to apply limits to the covariance.
        '''
        self.state = initial_state
        self.covariance = initial_covariance
        self.process_function = process_function
        self.process_noise = process_noise
        self.measurement_function = measurement_function
        self.measurement_noise = measurement_noise
        self.covariance_limit = covariance_limit

        assert self.state.shape[1] == 1
        assert self.state.shape[0] == self.covariance.shape[0]
        assert self.covariance.shape[0] == self.covariance.shape[1]

    def update_state(self, dt_s, process_function=None, process_noise=None):
        if process_function is None:
            process_function = self.process_function

        if process_noise is None:
            process_noise = self.process_noise

        sigma_points = self._get_sigma_points()

        # Equation 14.59
        xhat = [process_function(x, dt_s) for x in sigma_points]

        # Equation 14.60
        xhatminus = (1.0 / len(sigma_points)) * sum(xhat)

        # Equation 14.61

        # TODO jpieper: This does not handle states with unusual
        # domains, like angles, in the subtraction phase.
        Pminus = ((1.0 / len(sigma_points)) * sum([
                    numpy.dot((xhati - xhatminus),
                              (xhati - xhatminus).transpose())
                    for xhati in xhat]) +
                  dt_s * process_noise)

        self.state = xhatminus
        self.covariance = self._condition_covariance(Pminus)

    def update_measurement(self, measurement,
                           measurement_function=None, measurement_noise=None,
                           mahal_limit=None):
        assert measurement.shape[1] == 1

        if measurement_function is None:
            measurement_function = self.measurement_function

        if measurement_noise is None:
            measurement_noise = self.measurement_noise

        # Equation 14.62
        sigma_points = self._get_sigma_points()

        # Equation 14.63
        yhatin = [measurement_function(x) for x in sigma_points]

        # Equation 14.64
        yhat = (1.0 / len(yhatin)) * sum(yhatin)

        # Equation 14.65

        # TODO jpieper: This does not handle measurements with unusual
        # domains.
        Py = (1.0 / len(sigma_points)) * sum([
                numpy.dot((yhati - yhat),
                          (yhati - yhat).transpose())
                for yhati in yhatin]) + measurement_noise

        # Equation 14.66

        # TODO jpieper: This does not handle states or measurements
        # with unusual domains.
        Pxy = (1.0 / len(sigma_points)) * sum([
                numpy.dot((xi - self.state),
                          (yi - yhat).transpose())
                for xi, yi in zip(sigma_points, yhatin)])

        # Equation 14.67
        Py_inv = numpy.linalg.inv(Py)
        innovation = measurement - yhat
        if mahal_limit is not None:
            distance = numpy.dot(numpy.dot(innovation.transpose(), Py_inv),
                                 innovation)
            if distance[0, 0] > mahal_limit:
                return

        K = numpy.dot(Pxy, Py_inv)
        xplus = self.state + numpy.dot(K, innovation)
        Pplus = self.covariance - numpy.dot(numpy.dot(K, Py), K.transpose())

        self.state = xplus
        self.covariance = self._condition_covariance(Pplus)

    def _get_sigma_points(self):
        n = self.state.shape[0]
        delta = numpy.linalg.cholesky(n * self.covariance)
        return ([self.state + delta[:,x:x+1] for x in range(n)] +
                [self.state - delta[:,x:x+1] for x in range(n)])

    def _condition_covariance(self, P):
        result = P

        # Apply any user defined limiting.
        if self.covariance_limit:
            result = self.covariance_limit(result)

        # Force the result to be symmetric.
        result = 0.5 * (result + result.transpose())

        # And finally, force the result to be positive definite.
        result = make_positive_definite(result)

        return result
