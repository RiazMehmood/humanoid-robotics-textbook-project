import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '5ff'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', '5ba'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', 'a2b'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', 'c3c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', '156'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', '88c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', '000'),
    exact: true
  },
  {
    path: '/auth',
    component: ComponentCreator('/auth', '85d'),
    exact: true
  },
  {
    path: '/settings',
    component: ComponentCreator('/settings', 'b1c'),
    exact: true
  },
  {
    path: '/signout',
    component: ComponentCreator('/signout', '115'),
    exact: true
  },
  {
    path: '/',
    component: ComponentCreator('/', 'e5f'),
    exact: true
  },
  {
    path: '/',
    component: ComponentCreator('/', 'a99'),
    routes: [
      {
        path: '/',
        component: ComponentCreator('/', 'e31'),
        routes: [
          {
            path: '/',
            component: ComponentCreator('/', 'c31'),
            routes: [
              {
                path: '/intro',
                component: ComponentCreator('/intro', '4a7'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/module1/introduction',
                component: ComponentCreator('/module1/introduction', '468'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/module1/nodes-topics-services',
                component: ComponentCreator('/module1/nodes-topics-services', '97c'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/module1/python-agents-ros',
                component: ComponentCreator('/module1/python-agents-ros', '298'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/module1/ros2-fundamentals',
                component: ComponentCreator('/module1/ros2-fundamentals', '039'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/module1/urdf-humanoids',
                component: ComponentCreator('/module1/urdf-humanoids', '6f2'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/module2/gazebo-simulation',
                component: ComponentCreator('/module2/gazebo-simulation', '26d'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/module2/introduction',
                component: ComponentCreator('/module2/introduction', '592'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/module2/sensor-simulation',
                component: ComponentCreator('/module2/sensor-simulation', '259'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/module2/unity-rendering',
                component: ComponentCreator('/module2/unity-rendering', 'a40'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/module3/introduction',
                component: ComponentCreator('/module3/introduction', 'acd'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/module3/isaac-ros',
                component: ComponentCreator('/module3/isaac-ros', '0d4'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/module3/isaac-sim',
                component: ComponentCreator('/module3/isaac-sim', '6db'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/module3/nav2-path-planning',
                component: ComponentCreator('/module3/nav2-path-planning', '609'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/module4/capstone-project',
                component: ComponentCreator('/module4/capstone-project', '8f0'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/module4/cognitive-planning',
                component: ComponentCreator('/module4/cognitive-planning', '8b0'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/module4/introduction',
                component: ComponentCreator('/module4/introduction', 'f55'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/module4/voice-to-action',
                component: ComponentCreator('/module4/voice-to-action', '907'),
                exact: true,
                sidebar: "textbookSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
