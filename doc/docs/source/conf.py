# Configuration file for the Sphinx documentation builder.

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
