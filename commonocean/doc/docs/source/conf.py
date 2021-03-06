# Configuration file for the Sphinx documentation builder.

import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../../../..'))

# -- Project information

project = 'CommonOcean'
copyright = '2022, Maione'
author = 'Bruno'

release = '0.1'
version = '0.1.0'

# -- General configuration

extensions = [
    'sphinx.ext.duration',
    'sphinx.ext.doctest',
    'sphinx.ext.autodoc',
    'sphinx.ext.autosummary',
    'sphinx.ext.intersphinx',
    'matplotlib.sphinxext.plot_directive',
    'IPython.sphinxext.ipython_directive',
    'IPython.sphinxext.ipython_console_highlighting',
    'sphinx.ext.inheritance_diagram',
    'sphinx.ext.coverage',
    'sphinx.ext.mathjax',
    'sphinx.ext.viewcode',
]

intersphinx_mapping = {
    'python': ('https://docs.python.org/3/', None),
    'sphinx': ('https://www.sphinx-doc.org/en/master/', None),
}
intersphinx_disabled_domains = ['std']

templates_path = ['_templates']

# -- Options for HTML output
html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']
html_logo = "static_img_commonocean_logo_white.svg"
html_theme_options = {
    'logo_only': False,
    'display_version': False,
}
html_context = {
    "display_github": False, # Add 'Edit on Github' link instead of 'View page source'
    "last_updated": True,
    "commit": False,
}

# -- Options for EPUB output
epub_show_urls = 'footnote'
