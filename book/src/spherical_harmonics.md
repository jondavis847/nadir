## Intro
Since planetary bodies are almost spherical (but of course not quite), it makes sense that spherical harmonic models of varying degree for varying level of fidelity are utilized when modeling gravity, magnetic field, and other environmental models. Spherical harmonics are typically defined as some potential function, where values of a field at some point are defined as the gradient of the potential function at the point. 

The potential function is dependent on associated legendre polynomials that can be solved recursively through a number of algorithms. These recursion algorithms are typically explicitly defined in the literature for direct implementation. 

To get the gradient of the potential function, we need partial derivatives of the potential function with respect to our spherical coordinates, and these partial derivatives are not often provided directly. They also sometimes vary, or make assumptions and simplifications based on the model (i.e. \\(mu\\) might be included for earth gravity but not igrf, igrf has legendre polynomials in terms of colatitude instead of equatorial latitude, etc.). 

For a computer based library, my goal below is to derive the partial derivatives for reference, and to produce a generalized spherical harmonics library hat is applies to all earth based enviromental models.

## Conventions
### Geocentric Latitude vs Colatitude
Geocentric latitude is the latitude referenced from the equator at 0 degrees, the north hemisphere being 0 to 90 degrees and the southern hemisphere being 0 to -90 degrees. It is does not account for oblateness of the earth like geodetic latitude, and is directly related to ECEF position.

Colatitude is the latitude referenced from the north pole at 0 degrees, and going from 0 to \\(\pi\\) at the south pole.

In the following text I specify geocentric latitude as \\(\phi\\) and colatitude as \\(lambda\\). The relationship between the two is
\\[\phi = \lambda - \frac{\pi}{2}\\] Either can be used, make to pay attention to the literature for which one is used. Using one or the other just changes some sign conventions in the following derivations.

### Legendre Normalization
In the following derivations \\(\bar{P}_m^l\\) represents a normalized associated legendre polynomial of degree l and order m. Many texts use n for degree as well. Different normalizations are provided to accomplish goals like orthogonality or othonormality of the legendre polynomials. 

The coefficients for EGM96, C and S, are provided with full normalization using 
\\[\bar{P}^l_m = \sqrt{ \frac{\delta_{0m} (2l+1)(l-m)!}{2(l+m)!}}\\]
 
The coefficients for IGRF, g and h, are provided Schmidt-quasi normalization using
\\[\bar{P}^l_m = \sqrt{ (2 - \delta_{0m}) \frac{(l-m)!}{(l+m)!}} P^l_m\\] 

## Generalized spherical harmonics
Laplace's equation defines multidimensional potential equations where V is the potential.
\\[
\nabla^2 V = 0
\\]

Specifically solving in 3 dimensions and in spherical coordinates, Laplace's equation is 
\\[
\nabla^2 V = 
\frac{1}{r^2} \frac{\partial}{\partial r} \Bigl(r^2 \frac{\partial V}{\partial r}\Bigr) +
\frac{1}{r^2 \sin\theta} \frac{\partial}{\partial \theta} \Bigl(\sin\theta\ \frac{\partial V}{\partial \theta}\Bigr) +
\frac{1}{r^2 \sin^2\theta} \frac{\partial^2 V}{\partial \phi^2} = 0
\\]

Using separation of variables, the general solution to Laplace's equation is given as
\\[ V(r,\theta,\phi) = \sum_{\ell=0}^{\infty} \sum_{m=-\ell}^{\ell} \bigl[ A_{\ell m}r^{\ell} + B_{\ell m}r^{-(\ell+1)} \bigr] Y_{\ell}^{m}(\theta, phi)\\]

For the general solution, the term \\(A_{\ell m}r^{\ell}\\) only applies when inside the radius of the sphere, while \\(B_{\ell m}r^{-(\ell+1)}\\) applies when outside. For planetary based models outside of earth's radius, we can delete the A term. 

\\[ V(r,\theta,\phi) = \sum_{\ell=0}^{\infty} \sum_{m=-\ell}^{\ell} B_{\ell m}r^{-(\ell+1)} Y_{\ell}^{m}(\theta, phi)\\]

To solve for \\(Y_{\ell}^{m}(\theta, phi) \\), there is typically a complex solution and a real solution. Almost all literature on the space based models choose the real solution. In doing so, we limit the sum over m to the region 0 to l instead of -l to l, B gets broken into it's real and complex versions g and h, and 
\\[ Y_{\ell}^{m}(\theta, phi) = P_m^\ell(cos(\phi)) [ g_m^l cos(m\theta) + h_m^l sin(m\theta) ]\\] where \\(\phi\\) is the colatitude or 
\\[ Y_{\ell}^{m}(\theta, phi) = P_m^\ell(sin(\phi)) [ g_m^l cos(m\theta) + h_m^l sin(m\theta) ]\\] where \\(\phi\\) is the equatorial latitude.

Note also that B typically is manipualted to take out convenient factors depending on the use case. Doing so results in the following 2 well known potential functions for non spherical gravity and the magnetic field. 

### Non-spherical gravity
The total non spherical gravity is defined typically as

\\[V = \frac{\mu}{r} [1 + \sum_{l=1}^{\infty} \left(\frac{a}{r}\right) ^l \sum_{m=0}^{l} P^m_l (sin \phi) [C_m^l cos(m\theta) + S_m^l sin(m\theta)] ]\\]
Multiplying the 1 by \\(\frac{\mu}{r}\\) is the standard newtonian gravity. If instead we only want the acceleration perturbation due to non-spherical gravity, we simply remove the 1 and calculate the newtownian gravity separately.

\\[V = \frac{\mu}{r} \sum_{l=1}^{\infty} \left(\frac{a}{r}\right) ^l \sum_{m=0}^{l} P^m_l (sin \phi) [C_m^l cos(m\theta) + S_m^l sin(m\theta)] \\]

We will also need to be able to differentiate V by r, so we move the \\(\frac{1}{r}\\) from \\(\frac{\mu}{r}\\) term in to the sum, and multiple the whole thing by an additional \\(\frac{a}{a} =1\\) to get

\\[V = \frac{\mu}{a} \sum_{l=1}^{\infty} \left(\frac{a}{r}\right)^{l+1}  \sum_{m=0}^{l} P^m_l (sin \phi) [C_m^l cos(m\theta) + S_m^l sin(m\theta)] \\]

Just remember that this is only for the acceleration perturbation, and you might need to add back in newtonian gravity later.

### Magnetic field
The IGRF magnetic field potential is defined as 
\\[V(r) = a \sum_{l=1}^{\infty} \left(\frac{a}{r}\right) ^{l+1} \sum_{m=0}^{l} P^m_l (sin \phi) [g_m^l cos(m\theta) + h_m^l sin(m\theta) ]\\]

### General form
Taking the common equations, we have the general potential function
\\[V(r,k,a,g,h) = k \sum_{l=1}^{\infty} \left(\frac{a}{r}\right)^{l+1} \sum_{m=0}^{l} P^m_l (sin \phi) [g_m^l cos(m\theta) + h_m^l sin(m\theta) ]\\]

For non-spherical gravity, we calculated V(r,\\(\frac{\mu}{r}\\), a, C, S).
For IGRF, we calculated V(r, a, a, g, h).

# Calculating values
To solve for the acceleration due to gravity or the magnetic field, the values are the gradient of the potential function
\\[ g/b = -\nabla V = -\left( \frac{\partial V}{\partial r}, \frac{1}{r} \frac{\partial V}{\partial \phi}, \frac{1}{rsin\phi}\frac{\partial V}{\partial \theta}\right) \\]

## Calculating \\(\frac{\partial V}{\partial r}\\)
The only part of the potential function V depending on r is 
\\[\left(\frac{a}{r}\right) ^{l+1} \\] \\[ = a^{l+1}r^{-(l+1)} \\]
Solving the derivative we get
\\[\frac{d}{dr} a^{l+1}r^{-(l+1)} = -(l+1)(a^{l+1}) r^{-(l+2)} \\]
Factoring out \\(\frac{a}{r^2}\\) we get
\\[ =-\frac{a}{r^2}(l+1)(a^l)r^{-l} \\]
\\[ =-\frac{a}{r^2}(l+1)\left(\frac{a}{r}\right)^l\\]

The final equation for latitude is 
\\[\frac{\partial V}{\partial r} = - \frac{ka}{r^2} \sum_{l=1}^{\infty} (l+1) \left(\frac{a}{r}\right)^l \sum_{m=0}^{l} \bar{P}^m_l (sin \phi) \left[g^m_l cos(m\theta) + h^m_l sin(m\theta) \right]\\]
or for colatitude
\\[\frac{\partial V}{\partial r} = -\frac{ka}{r^2} \sum_{l=1}^{\infty} (l+1) \left(\frac{a}{r}\right)^l \sum_{m=0}^{l} \bar{P}^m_l (cos \lambda) \left[g^m_l cos(m\theta) + h^m_l sin(m\theta) \right]\\]
## Calculating \\(\frac{\partial V}{\partial \phi}\\)

Since this partial does not depend on r, we can take out the \\(\frac{a}{r}\\) immediately so that 
\\[V(r,k,a,g,h) = \frac{ka}{r} \sum_{l=1}^{\infty} \left(\frac{a}{r}\right)^l \sum_{m=0}^{l} P^m_l (sin \phi) [g_m^l cos(m\theta) + h_m^l sin(m\theta) ]\\]


The only part of the potential function V depending on \\(\phi\\) is \\(  \bar{P}_m^l (sin \phi) \\). We use the chain rule to get the derivative as

\\[\frac{d}{d\phi} \bar{P}_m^l (sin \phi)\\]
\\[ = \frac{d}{dx} \bar{P}_m^l (x) \frac{d}{d\phi} sin \phi\\]
\\[ = cos\phi \frac{d}{dx} \bar{P}_m^l (x)\\]

The term \\(\frac{d}{dx} {P}^l_m(x)\\) is calculated with recursion relations while calculating \\({P}^l_m(sin\phi)\\) since there are recursive versions that depend only on \\(\bar{P}^l_m\\) and its previously calculated values.

Remembering that \\(x = sin\phi\\), and from \\(1 = cos^2\phi + sin^2\phi => cos\phi = \sqrt{1 - sin^2\phi} = \sqrt{1 - x^2}\\), we get the final derivative for latitude as 
\\[\sqrt{1 - x^2} \frac{d}{dx} \bar{P}_m^l (x) \\] 

Similarly for colatitue, remembering that \\(x = cos\phi\\), and from \\(1 = cos^2\phi + sin^2\phi => sin\phi = \sqrt{1 - cos^2\phi} = \sqrt{1 - x^2}\\), we get the final derivative for latitude as 
\\[-\sqrt{1 - x^2} \frac{d}{dx} \bar{P}_m^l (x) \\] 

The final equation for latitude is 
\\[\frac{\partial V}{\partial \phi} = \frac{ka}{r} \sum_{l=1}^{\infty} \left(\frac{a}{r}\right)^l \sum_{m=0}^{l} \sqrt{1 - x^2} \frac{d}{dx} P^m_l (x) [g^m_l (t) cos(m\theta) + h^m_l (t) sin(m\theta) ] \\]
or for colatitude
\\[\frac{\partial V}{\partial \lambda} = -\frac{ka}{r} \sum_{l=1}^{\infty} \left(\frac{a}{r}\right)^l \sum_{m=0}^{l} \sqrt{1 - x^2} \frac{d}{dx} P^m_l (x) [g^m_l (t) cos(m\theta) + h^m_l (t) sin(m\theta) ] \\]

## Calculating \\(\frac{\partial V}{\partial \theta}\\)

Similarly, since this partial does not depend on r, we can take out the \\(\frac{a}{r}\\) immediately so that 
\\[V(r,k,a,g,h) = \frac{ka}{r} \sum_{l=1}^{\infty} \left(\frac{a}{r}\right)^l \sum_{m=0}^{l} P^m_l (sin \phi) [g_m^l cos(m\theta) + h_m^l sin(m\theta) ]\\]

The only part of the potential function V depending on \\(\theta\\) is \\([g^m_l (t) cos(m\theta) + h^m_l (t) sin(m\theta) ]\\).
\\[\frac{d}{d\theta} [g^m_l (t) cos(m\theta) + h^m_l (t) sin(m\theta) ] = [-m g^m_l (t) sin(m\theta) + m h^m_l (t) cos(m\theta) ] \\]
\\[ = m [ h^m_l (t) cos(m\theta) - g^m_l (t) sin(m\theta)] \\]

The final equation for latitude is
\\[\frac{\partial V}{\partial \theta} = \frac{ka}{r} \sum_{l=1}^{\infty} \left(\frac{a}{r}\right) ^l \sum_{m=0}^{l} m P^m_l (sin \phi) [ h^m_l (t) cos(m\theta) - g^m_l (t) sin(m\theta)]\\]

or for colatitude
\\[\frac{\partial V}{\partial \theta} = \frac{ka}{r} \sum_{l=1}^{\infty} \left(\frac{a}{r}\right) ^l \sum_{m=0}^{l} m P^m_l (cos \lambda) [ h^m_l (t) cos(m\theta) - g^m_l (t) sin(m\theta)]\\]

## References
Gottlieb: https://ntrs.nasa.gov/api/citations/19940025085/downloads/19940025085.pdf
