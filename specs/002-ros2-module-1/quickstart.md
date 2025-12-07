# Quickstart: Building and Running the Docusaurus Site Locally

This guide provides instructions on how to set up the development environment and run the Docusaurus site for this project.

## Prerequisites

- Node.js (version 16.x or later)
- npm (version 8.x or later)

## Installation

1. Clone the repository:
   ```bash
   git clone <repository-url>
   ```
2. Navigate to the project directory:
   ```bash
   cd giaic-hackathonn
   ```
3. Install the dependencies:
   ```bash
   npm install
   ```

## Running the Development Server

To start the Docusaurus development server, run the following command from the project root:

```bash
npm start
```

This will open the site in your browser at `http://localhost:3000`. The site will automatically reload if you make changes to the content in the `docs` directory.

## Building the Site

To create a static build of the site, run the following command from the project root:

```bash
npm run build
```

The built files will be located in the `build` directory. These files can be deployed to any static web hosting service.
