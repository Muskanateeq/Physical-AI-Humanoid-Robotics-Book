# Implementation Plan: Docusaurus Homepage for Physical AI Humanoid Robotics Course

This document provides a detailed, step-by-step plan to implement the new Docusaurus homepage based on the approved specification (`spec-homepage.md`).

## Phase 1: Project Setup & Configuration

This phase ensures the project's structure and configuration are correctly set up for the new homepage.

1.  **Update `docusaurus.config.js`:**
    - **Action:** Modify the site's metadata.
    - **Details:**
        - Set `title` to `Physical AI Humanoid Robotics Course`.
        - Set `tagline` to `Build, train, and deploy the next generation of intelligent humanoid robots.`.
        - Update `navbar.title` to `Physical AI Humanoid Robotics`.
        - Ensure the `navbar` `items` array points the primary "Course Modules" link to the first page of the docs (e.g., `/docs/module-1-ros2/ros2-basics`).
        - Update the `footer` copyright notice.
        - Verify `theme.customCss` points to `src/css/custom.css`.

2.  **Clean Up `src/pages/`:**
    - **Action:** Remove the default markdown page.
    - **Details:** Delete the `src/pages/markdown-page.md` file, as it is no longer required. The primary homepage will be `src/pages/index.js`.

3.  **Verify Sidebar Configuration:**
    - **Action:** Check `sidebars.js`.
    - **Details:** Ensure that the `tutorialSidebar` is correctly configured to auto-generate the sidebar from the `docs/` directory. No changes should be needed based on the current file content.

## Phase 2: Theming & Styling (`custom.css`)

This phase focuses on implementing the visual identity and dark theme of the site. All CSS will be written in `src/css/custom.css`.

1.  **Define CSS Variables:**
    - **Action:** At the root of the CSS file (`:root` and `[data-theme='dark']`), define the color palette from the specification.
    - **Details:**
        - `--ifm-color-primary-darkest`: `#0D1117` (Deep Space Blue)
        - `--ifm-font-color-base`: `#F0F6FC` (Off-white)
        - `--ifm-heading-color`: `#F0F6FC`
        - `--accent-cyan`: `#00E5FF`
        - `--accent-magenta`: `#FF00FF`
        - `--accent-silver`: `#C0C0C0`
        - `--secondary-text-color`: `#8B949E`

2.  **Implement Global Styles:**
    - **Action:** Set the base typography and background for the entire site.
    - **Details:**
        - Import the `Roboto` and `Open Sans` fonts from Google Fonts at the top of the file.
        - Set the `body` font to `Open Sans`.
        - Set the `h1`, `h2`, `h3`, etc. font to `Roboto`.
        - Set the default `background-color` of the `body` to the primary background color.

3.  **Create Utility Classes:**
    - **Action:** Define reusable CSS classes for common patterns.
    - **Details:**
        - Create a `.container` class for centered content.
        - Create classes for section padding (e.g., `.section--padding`).
        - Create classes for button styles (`.button--primary`, `.button--secondary`). The primary button will have a cyan/magenta gradient with a glow effect.

## Phase 3: Asset Generation & Integration

This phase involves creating or sourcing the visual assets needed for the homepage.

1.  **Hero Image:**
    - **Action:** Create a placeholder hero image.
    - **Details:** Generate a dark, abstract background with blue and purple hues. A simple generative art tool or a gradient with noise can be used. A placeholder representing a humanoid robot (even a silhouette) can be added. Save as `static/img/hero-background.jpg`.

2.  **Module Icons:**
    - **Action:** Create icons for the four course modules.
    - **Details:** Source simple, white or cyan SVG icons that represent:
        - **ROS2:** Interconnected gears.
        - **Simulation:** A computer monitor or a 3D cube.
        - **AI Brain:** A brain or neural network icon.
        - **VLA:** An eye, speech bubble, and hand combined.
    - Save these as SVGs in `static/img/icons/`.

3.  **Other Visuals:**
    - **Action:** Create a placeholder for the "Course Overview" section visual.
    - **Details:** Generate an abstract image of a robotic hand or circuitry. Save as `static/img/overview-visual.png`.

## Phase 4: Homepage Component Implementation (`index.js`)

This is the main implementation phase, where the `src/pages/index.js` file is rewritten to build the homepage using React components.

1.  **Main Homepage Layout (`Homepage` function):**
    - **Action:** Create the main layout component.
    - **Details:** This function will return a `<Layout>` component containing all the homepage sections in order. It will set the `title` and `description` for the page.

2.  **Hero Section Component (`HomepageHero`):**
    - **Action:** Build the hero section.
    - **Details:**
        - Use a `<header>` element with a background image pointing to `static/img/hero-background.jpg`.
        - Add the `h1` title and `p` subtitle from the spec.
        - Add a `<Link>` component styled as the primary CTA button, linking to the first page of the course docs.

3.  **Overview Section Component (`CourseOverview`):**
    - **Action:** Build the "Welcome to the Future" section.
    - **Details:**
        - Create a section with a two-column layout (e.g., using Flexbox).
        - Place the heading and paragraphs in the left column.
        - Place an `<img>` tag in the right column for `static/img/overview-visual.png`.

4.  **Modules Section Component (`CourseModules`):**
    - **Action:** Build the "What You Will Master" grid.
    - **Details:**
        - Create a `modules` array of objects, where each object contains the `title`, `icon` (path to SVG), and `description` for a module.
        - Map over this array to render a `ModuleCard` component for each item.
        - Style the container as a 2x2 grid.

5.  **Module Card Component (`ModuleCard`):**
    - **Action:** Create the reusable card component for the modules section.
    - **Details:**
        - This component will accept `title`, `icon`, and `description` as props.
        - The card will have a border, a subtle background color, and a hover effect (glow, scale).
        - Inside the card, it will display the icon, title, and description text.

6.  **Features Section Component (`KeyFeatures`):**
    - **Action:** Build the "Course Features" list.
    - **Details:**
        - Create a `features` array of strings.
        - Map over the array to render a list item for each feature.
        - Use a custom CSS list style or an inline SVG to create the `✓` checkmark.

7.  **Final CTA Component (`FinalCTA`):**
    - **Action:** Build the final call-to-action section.
    - **Details:**
        - A simple section with a `h2` heading and the main CTA button, centered.

## Phase 5: Final Assembly & Review

1.  **Import and Assemble:**
    - **Action:** In `src/pages/index.js`, import all the created components (`HomepageHero`, `CourseOverview`, etc.).
    - **Details:** Render them in the correct order within the main `Homepage` component's `<Layout>`.

2.  **Testing:**
    - **Action:** Run the Docusaurus development server (`npm run start` or `yarn start`).
    - **Details:**
        - Review the homepage in a browser.
        - Check for visual consistency with the spec.
        - Test all links and interactive elements.
        - Verify the responsive design on different screen sizes.

This plan provides a clear path to transform the specification into a fully functional and visually appealing homepage.
