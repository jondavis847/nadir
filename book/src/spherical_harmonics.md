# IGRF
The potential is given as
\\[V(r) = a \sum_{n=1}^{\infty} (\frac{a}{r}) ^{n+1} \sum_{m=0}^{n} \bar{P}^m_n (sin \theta) [g^m_n (t) cos(m\phi) + h^m_n (t) sin(m\phi) ]\\]

Where g and h are the IGRF gauss coefficients found at https://www.ngdc.noaa.gov/IAGA/vmod/coeffs/igrf14coeffs.txt, \\(\bar{P}\\) is the Schmidt normalized associated legendre polynomials, the Schmidt normalization being 
\\[\bar{P}^m_n = \sqrt{ (2 - \delta_{0m}) \frac{(n-m)!}{(n+m)!}} P^m_n\\]

The magnetic field in the \\((r,\theta,\phi)\\) frame is 
\\[b = -\nabla V = \\]

